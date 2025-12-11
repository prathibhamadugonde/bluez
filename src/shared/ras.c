// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2025  Qualcomm Incorporated. All rights reserved.
 *
 */

#define _GNU_SOURCE
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <glib.h>

#include "bluetooth/bluetooth.h"
#include "bluetooth/uuid.h"

#include "src/shared/queue.h"
#include "src/shared/util.h"
#include "src/shared/timeout.h"
#include "src/log.h"
#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-server.h"
#include "src/shared/gatt-client.h"
#include "src/shared/ras.h"

#define RAS_UUID16			0x185B
#define RAS_FEATURES_UUID		0x2C14
#define RAS_REALTIME_DATA_UUID		0x2C15
#define RAS_ONDEMAND_DATA_UUID		0x2C16
#define RAS_CONTROL_POINT_UUID		0x2C17
#define RAS_DATA_READY_UUID		0x2C18
#define RAS_DATA_OVERWRITTEN_UUID	0x2C19
#define RAS_TOTAL_NUM_HANDLES		18
/* Ranging Service */
struct ras {
    struct bt_ras_db *rasdb;
    /* Service and characteristic attributes */
    struct gatt_db_attribute *svc;
    struct gatt_db_attribute *feat_chrc;
    struct gatt_db_attribute *realtime_chrc;
    struct gatt_db_attribute *realtime_chrc_ccc;
    struct gatt_db_attribute *ondemand_chrc;
    struct gatt_db_attribute *cp_chrc;
    struct gatt_db_attribute *ready_chrc;
    struct gatt_db_attribute *overwritten_chrc;
};

struct bt_ras_db {
    struct gatt_db *db;
    struct ras *ras;
};

struct bt_ras {
    int ref_count;
    struct bt_ras_db *lrasdb;
    struct bt_ras_db *rrasdb;
    struct bt_gatt_client *client;
    struct bt_att *att;

    unsigned int idle_id;

    struct queue *notify;
    struct queue *pending;
    struct queue *ready_cbs;

    void *user_data;
};

static struct queue *ras_db;
static struct queue *bt_ras_cbs;
static struct queue *sessions;

struct bt_ras_cb {
    unsigned int id;
    bt_ras_func_t attached;
    bt_ras_func_t detached;
    void *user_data;
};

typedef void (*ras_func_t)(struct bt_ras *bt_ras, bool success,
        uint8_t att_ecode, const uint8_t *value,
        uint16_t length, void *user_data);

struct bt_ras_pending {
    unsigned int id;
    struct bt_ras *bt_ras;
    ras_func_t func;
    void *userdata;
};

struct bt_ras_ready {
    unsigned int id;
    bt_ras_ready_func_t func;
    bt_ras_destroy_func_t destroy;
    void *data;
};

typedef void (*ras_notify_t)(struct bt_ras *bt_ras, uint16_t value_handle,
        const uint8_t *value, uint16_t length,
        void *user_data);

struct bt_ras_notify {
    unsigned int id;
    struct bt_ras *bt_ras;
    ras_notify_t func;
    void *user_data;
};

static bool real_time_enabled = false;
static bool on_demand_enabled = false;
struct gatt_db_attribute *global_real_time_char = NULL;
struct gatt_db_attribute *global_on_demand_char = NULL;
struct gatt_db_attribute *global_data_ready_char = NULL;
struct gatt_db_attribute *global_data_overwritten_char = NULL;
struct gatt_db_attribute *global_control_point_char = NULL;

static struct bt_ras_db *ras_get_rasdb(struct bt_ras *bt_ras)
{
	if (!bt_ras)
		return NULL;

	if (bt_ras->lrasdb)
		return bt_ras->lrasdb;

	return NULL;
}

struct ras *btras_get_ras(struct bt_ras *bt_ras)
{
	if (!bt_ras)
		return NULL;

	if (bt_ras->rrasdb->ras)
		return bt_ras->rrasdb->ras;

	bt_ras->rrasdb->ras = new0(struct ras, 1);
	bt_ras->rrasdb->ras->rasdb = bt_ras->rrasdb;

	return bt_ras->rrasdb->ras;
}

static void bt_ras_remote_client_attached(void *data, void *user_data)
{
	struct bt_ras_cb *cb = data;
	struct bt_ras *bt_ras = user_data;

	cb->attached(bt_ras, cb->user_data);
}

static void bt_ras_remote_client_detached(void *data, void *user_data)
{
	struct bt_ras_cb *cb = data;
	struct bt_ras *bt_ras = user_data;

	cb->detached(bt_ras, cb->user_data);
}

static void ras_detached(void *data, void *user_data)
{
	struct bt_ras_cb *cb = data;
	struct bt_ras *bt_ras = user_data;

	cb->detached(bt_ras, cb->user_data);
}

void bt_ras_detach(struct bt_ras *bt_ras)
{
	if (!queue_remove(sessions, bt_ras))
		return;

	bt_gatt_client_idle_unregister(bt_ras->client, bt_ras->idle_id);
	bt_gatt_client_unref(bt_ras->client);
	bt_ras->client = NULL;

	queue_foreach(bt_ras_cbs, ras_detached, bt_ras);
}

static void ras_db_free(void *data)
{
	struct bt_ras_db *rasdb = data;

	if (!rasdb)
		return;

	gatt_db_unref(rasdb->db);

	free(rasdb->ras);
	free(rasdb);
}

static void ras_ready_free(void *data)
{
	struct bt_ras_ready *ready = data;

	if (ready->destroy)
		ready->destroy(ready->data);

	free(ready);
}

static void ras_free(void *data)
{
	struct bt_ras *bt_ras = data;

	bt_ras_detach(bt_ras);

	ras_db_free(bt_ras->rrasdb);

	queue_destroy(bt_ras->notify, free);
	queue_destroy(bt_ras->pending, NULL);
	queue_destroy(bt_ras->ready_cbs, ras_ready_free);

	free(bt_ras);
}

bool bt_ras_set_user_data(struct bt_ras *bt_ras, void *user_data)
{

	if (!bt_ras)
		return false;

	bt_ras->user_data = user_data;

	return true;
}

static bool ras_db_match(const void *data, const void *match_data)
{
	const struct bt_ras_db *rasdb = data;
	const struct gatt_db *db = match_data;

	return (rasdb->db == db);
}

struct bt_att *bt_ras_get_att(struct bt_ras *bt_ras)
{
	if (!bt_ras)
		return NULL;

	if (bt_ras->att)
		return bt_ras->att;

	return bt_gatt_client_get_att(bt_ras->client);
}

struct bt_ras *bt_ras_ref(struct bt_ras *bt_ras)
{
	if (!bt_ras)
		return NULL;

	__sync_fetch_and_add(&bt_ras->ref_count, 1);

	return bt_ras;
}

void bt_ras_unref(struct bt_ras *bt_ras)
{
	if (!bt_ras)
		return;

	if (__sync_sub_and_fetch(&bt_ras->ref_count, 1))
		return;

	ras_free(bt_ras);
}

static void ras_disconnected(int err, void *user_data)
{
	/* called only when this device is acting a a server */
	struct bt_ras *bt_ras = user_data;

	DBG("RAS %p disconnected err %d", bt_ras, err);

	bt_ras_detach(bt_ras);
	queue_foreach(bt_ras_cbs, bt_ras_remote_client_detached, bt_ras);
}

static struct bt_ras *ras_get_session(struct bt_att *att, struct gatt_db *db)
{
	const struct queue_entry *entry;
	struct bt_ras *bt_ras;

	for (entry = queue_get_entries(sessions); entry; entry = entry->next) {
		struct bt_ras *bt_ras = entry->data;

		if (att == bt_ras_get_att(bt_ras))
			return bt_ras;
	}

	bt_ras = bt_ras_new(db, NULL);
	bt_ras->att = att;

	queue_foreach(bt_ras_cbs, bt_ras_remote_client_attached, bt_ras);

	bt_att_register_disconnect(att, ras_disconnected, bt_ras, NULL);

	bt_ras_attach(bt_ras, NULL);

	if (!sessions)
		sessions = queue_new();

	queue_push_tail(sessions, bt_ras);

	return bt_ras;
}

/* -------------------------------------------------------------------------
 * Characteristic callbacks
 * ------------------------------------------------------------------------- */
static void ras_features_read_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
        DBG(" ");
	/* Feature mask: bits 0‑2 set (real‑time, retrieve lost, abort) */
	/*Set Real-time Ranging data*/
	uint8_t value[4] = { 0x01, 0x00, 0x00, 0x00 };
	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void ras_realtime_read_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
	/* No static read data – real‑time data is pushed via notifications */
	gatt_db_attribute_read_result(attrib, id, 0, NULL, 0);
}

static void ras_ondemand_read_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
        DBG(" ");
	/* No static read data – on‑demand data is pushed via notifications */
	gatt_db_attribute_read_result(attrib, id, 0, NULL, 0);
}

/* Control Point – parses Op‑Code and acts on queued data */
static void ras_control_point_write_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				const uint8_t *value, size_t len,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
        DBG(" ");
}

/* Data Ready – returns the latest ranging counter */
static void ras_data_ready_read_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
	uint16_t counter = 0;
	uint8_t value[2];
	put_le16(counter, value);
        DBG(" ");
	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

/* Data Overwritten – for stub returns zero */
static void ras_data_overwritten_read_cb(struct gatt_db_attribute *attrib,
				unsigned int id, uint16_t offset,
				uint8_t opcode, struct bt_att *att,
				void *user_data)
{
	uint8_t value[2] = { 0x00, 0x00 };
        DBG(" ");
	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

/* -------------------------------------------------------------------------
 * Service registration – store attribute pointers
 * ------------------------------------------------------------------------- */
static struct ras *register_ras_service(struct gatt_db *db)
{
	struct ras *ras;
        struct gatt_db_attribute *service;
        bt_uuid_t uuid;

	if (!db)
		return NULL;

	ras = new0(struct ras, 1);

	/* Primary RAS service */
        bt_uuid16_create(&uuid, RAS_UUID16);
        service = gatt_db_add_service(db, &uuid, true, RAS_TOTAL_NUM_HANDLES);
        if (!service)
	{
		DBG("ras profile uuid is not added");
                return false;
	}
        ras->svc = service;

	/* RAS Features */
        bt_uuid16_create(&uuid, RAS_FEATURES_UUID);
        ras->feat_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_READ | BT_ATT_PERM_READ_ENCRYPT,
                        BT_GATT_CHRC_PROP_READ,
                        ras_features_read_cb, NULL, ras);

        /* Real‑time Ranging Data */
        bt_uuid16_create(&uuid, RAS_REALTIME_DATA_UUID);
        ras->realtime_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_READ | BT_ATT_PERM_READ_ENCRYPT,
                        BT_GATT_CHRC_PROP_NOTIFY | BT_GATT_CHRC_PROP_INDICATE,
                        NULL, NULL, ras);

	ras->realtime_chrc_ccc = gatt_db_service_add_ccc(ras->svc,
			BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

	/* On‑demand Ranging Data */
        bt_uuid16_create(&uuid, RAS_ONDEMAND_DATA_UUID);
        ras->ondemand_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_READ | BT_ATT_PERM_READ_ENCRYPT,
                        BT_GATT_CHRC_PROP_NOTIFY | BT_GATT_CHRC_PROP_INDICATE,
                        ras_ondemand_read_cb, NULL, ras);

	gatt_db_service_add_ccc(ras->svc,
                        BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

        /* RAS Control Point */
        bt_uuid16_create(&uuid, RAS_CONTROL_POINT_UUID);
        ras->cp_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_WRITE | BT_ATT_PERM_WRITE_ENCRYPT,
                        BT_GATT_CHRC_PROP_WRITE_WITHOUT_RESP |
                        BT_GATT_CHRC_PROP_INDICATE,
                        NULL, ras_control_point_write_cb, ras);
        gatt_db_service_add_ccc(ras->svc,
                        BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

        /* RAS Data Ready */
        bt_uuid16_create(&uuid, RAS_DATA_READY_UUID);
        ras->ready_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_READ | BT_ATT_PERM_READ_ENCRYPT,
                        BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_NOTIFY |
                        BT_GATT_CHRC_PROP_INDICATE,
                        ras_data_ready_read_cb, NULL, ras);
        gatt_db_service_add_ccc(ras->svc,
                        BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

        /* RAS Data Overwritten */
        bt_uuid16_create(&uuid, RAS_DATA_OVERWRITTEN_UUID);
        ras->overwritten_chrc = gatt_db_service_add_characteristic(ras->svc, &uuid,
                        BT_ATT_PERM_READ | BT_ATT_PERM_READ_ENCRYPT,
                        BT_GATT_CHRC_PROP_READ | BT_GATT_CHRC_PROP_NOTIFY |
                        BT_GATT_CHRC_PROP_INDICATE,
                        ras_data_overwritten_read_cb, NULL, ras);

	gatt_db_service_add_ccc(ras->svc,
                        BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

        /* Activate the service */
	gatt_db_service_set_active(ras->svc, true);

	return ras;
}

static struct bt_ras_db *ras_db_new(struct gatt_db *db)
{
    struct bt_ras_db *rasdb;
    struct ras *ras;

    if(!db)
	    return NULL;

    rasdb = new0(struct bt_ras_db, 1);
    if (!rasdb) {
	    DBG("RAS DB probe failed: memory allocation failed");
	    return NULL;
    }

    rasdb->db = gatt_db_ref(db);

    if (!ras_db)
	    ras_db = queue_new();

    rasdb->ras = register_ras_service(db);
    rasdb->ras->rasdb = rasdb;

    queue_push_tail(ras_db, rasdb);

    return rasdb;
}

static struct bt_ras_db *ras_get_db(struct gatt_db *db)
{
	struct bt_ras_db *rasdb;

	rasdb = queue_find(ras_db, ras_db_match, db);
	if (rasdb)
		return rasdb;

	return ras_db_new(db);
}

void bt_ras_add_db(struct gatt_db *db)
{
	ras_db_new(db);
}

unsigned int bt_ras_register(bt_ras_func_t attached, bt_ras_func_t detached,
							  void *user_data)
{
	struct bt_ras_cb *cb;
	static unsigned int id;

	if (!attached && !detached)
		return 0;

	if (!bt_ras_cbs)
		bt_ras_cbs = queue_new();

	cb = new0(struct bt_ras_cb, 1);
	cb->id = ++id ? id : ++id;
	cb->attached = attached;
	cb->detached = detached;
	cb->user_data = user_data;

	queue_push_tail(bt_ras_cbs, cb);

	return cb->id;
}

static bool match_id(const void *data, const void *match_data)
{
	const struct bt_ras_cb *cb = data;
	unsigned int id = PTR_TO_UINT(match_data);

	return (cb->id == id);
}

bool bt_ras_unregister(unsigned int id)
{
	struct bt_ras_cb *cb;

	cb = queue_remove_if(bt_ras_cbs, match_id, UINT_TO_PTR(id));
	if (!cb)
		return false;

	free(cb);

	return true;
}

struct bt_ras *bt_ras_new(struct gatt_db *ldb, struct gatt_db *rdb)
{
	struct bt_ras *bt_ras;
	struct bt_ras_db *rasdb;

	if (!ldb)
		return NULL;

	rasdb = ras_get_db(ldb);
	if (!rasdb)
		return NULL;

	bt_ras = new0(struct bt_ras, 1);
	bt_ras->lrasdb = rasdb;
	bt_ras->pending = queue_new();
	bt_ras->ready_cbs = queue_new();
	bt_ras->notify = queue_new();

	if (!rdb)
		goto done;

	rasdb = new0(struct bt_ras_db, 1);
	rasdb->db = gatt_db_ref(rdb);

	bt_ras->rrasdb = rasdb;

done:
	bt_ras_ref(bt_ras);

	return bt_ras;
}

static void ras_pending_destroy(void *data)
{
	struct bt_ras_pending *pending = data;
	struct bt_ras *bt_ras = pending->bt_ras;

	if (queue_remove_if(bt_ras->pending, NULL, pending))
		free(pending);
}

static void ras_pending_complete(bool success, uint8_t att_ecode,
				const uint8_t *value, uint16_t length,
				void *user_data)
{
	struct bt_ras_pending *pending = user_data;

	if (pending->func)
		pending->func(pending->bt_ras, success, att_ecode, value, length,
					  pending->userdata);
}

static void ras_register(uint16_t att_ecode, void *user_data)
{
	struct bt_ras_notify *notify = user_data;

	if (att_ecode)
		DBG("RAS register failed 0x%04x", att_ecode);
}

static void ras_notify(uint16_t value_handle, const uint8_t *value,
				uint16_t length, void *user_data)
{
	struct bt_ras_notify *notify = user_data;

	if (notify->func)
		notify->func(notify->bt_ras, value_handle, value, length,
					 notify->user_data);
}

static void ras_notify_destroy(void *data)
{
	struct bt_ras_notify *notify = data;
	struct bt_ras *bt_ras = notify->bt_ras;

	if (queue_remove_if(bt_ras->notify, NULL, notify))
		free(notify);
}

static unsigned int bt_ras_register_notify(struct bt_ras *bt_ras,
					uint16_t value_handle,
					ras_notify_t func,
					void *user_data)
{
	struct bt_ras_notify *notify;

	notify = new0(struct bt_ras_notify, 1);
	notify->bt_ras = bt_ras;
	notify->func = func;
	notify->user_data = user_data;

	notify->id = bt_gatt_client_register_notify(bt_ras->client,
					value_handle, ras_register,
					ras_notify, notify,
					ras_notify_destroy);
	if (!notify->id) {
		DBG("Unable to register for notifications");
		free(notify);
		return 0;
	}

	queue_push_tail(bt_ras->notify, notify);

	return notify->id;
}

static void foreach_ras_char(struct gatt_db_attribute *attr, void *user_data)
{
	struct bt_ras *bt_ras = user_data;
	uint16_t value_handle;
	bt_uuid_t uuid, uuid_features, uuid_realtime, uuid_ondemand, uuid_cp, uuid_dataready, uuid_overwritten;
	struct ras *ras;

	if (!gatt_db_attribute_get_char_data(attr, NULL, &value_handle,
			NULL, NULL, &uuid))
		return;

	bt_uuid16_create(&uuid_features, RAS_FEATURES_UUID);
	bt_uuid16_create(&uuid_realtime, RAS_REALTIME_DATA_UUID);
	bt_uuid16_create(&uuid_ondemand, RAS_ONDEMAND_DATA_UUID);
	bt_uuid16_create(&uuid_cp, RAS_CONTROL_POINT_UUID);
	bt_uuid16_create(&uuid_dataready, RAS_DATA_READY_UUID);
	bt_uuid16_create(&uuid_overwritten, RAS_DATA_OVERWRITTEN_UUID);

	if (!bt_uuid_cmp(&uuid, &uuid_features)) {
		DBG(" Features characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->feat_chrc)
			return;

		ras->feat_chrc = attr;

	}

	if (!bt_uuid_cmp(&uuid, &uuid_realtime)) {
		DBG(" Real Time Data characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->realtime_chrc)
			return;

		ras->realtime_chrc = attr;

	}

	if (!bt_uuid_cmp(&uuid, &uuid_ondemand)) {
		DBG("Ondemand Data characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->ondemand_chrc)
			return;

		ras->ondemand_chrc = attr;

	}

	if (!bt_uuid_cmp(&uuid, &uuid_cp)) {
		DBG(" Contorl Point characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->cp_chrc)
			return;

		ras->cp_chrc = attr;

	}

	if (!bt_uuid_cmp(&uuid, &uuid_dataready)) {
		DBG(" Data Ready characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->ready_chrc)
			return;

		ras->ready_chrc = attr;

	}

	if (!bt_uuid_cmp(&uuid, &uuid_overwritten)) {
		DBG(" Overwritten characteristic found: handle 0x%04x",
				value_handle);

		ras = btras_get_ras(bt_ras);
		if (!ras || ras->overwritten_chrc)
			return;

		ras->overwritten_chrc = attr;

	}
}

static void foreach_ras_service(struct gatt_db_attribute *attr,
					void *user_data)
{
	struct bt_ras *bt_ras = user_data;
	struct ras *ras = btras_get_ras(bt_ras);

	ras->svc = attr;

	gatt_db_service_set_claimed(attr, true);

	gatt_db_service_foreach_char(attr, foreach_ras_char, bt_ras);
}

unsigned int bt_ras_ready_register(struct bt_ras *bt_ras,
				bt_ras_ready_func_t func, void *user_data,
				bt_ras_destroy_func_t destroy)
{
	struct bt_ras_ready *ready;
	static unsigned int id;

	DBG("bt_ras_ready_register_Entry\n");
	if (!bt_ras)
		return 0;

	ready = new0(struct bt_ras_ready, 1);
	ready->id = ++id ? id : ++id;
	ready->func = func;
	ready->destroy = destroy;
	ready->data = user_data;

	queue_push_tail(bt_ras->ready_cbs, ready);

	return ready->id;
}

static bool match_ready_id(const void *data, const void *match_data)
{
	const struct bt_ras_ready *ready = data;
	unsigned int id = PTR_TO_UINT(match_data);

	return (ready->id == id);
}

bool bt_ras_ready_unregister(struct bt_ras *bt_ras, unsigned int id)
{
	struct bt_ras_ready *ready;

	ready = queue_remove_if(bt_ras->ready_cbs, match_ready_id,
						UINT_TO_PTR(id));
	if (!ready)
		return false;

	ras_ready_free(ready);

	return true;
}

static struct bt_ras *bt_ras_ref_safe(struct bt_ras *bt_ras)
{
	if (!bt_ras || !bt_ras->ref_count)
		return NULL;

	return bt_ras_ref(bt_ras);
}

static void ras_notify_ready(struct bt_ras *bt_ras)
{
	const struct queue_entry *entry;

	if (!bt_ras_ref_safe(bt_ras))
		return;

	for (entry = queue_get_entries(bt_ras->ready_cbs); entry;
							entry = entry->next) {
		struct bt_ras_ready *ready = entry->data;

		ready->func(bt_ras, ready->data);
	}

	bt_ras_unref(bt_ras);
}

static void ras_idle(void *data)
{
	struct bt_ras *bt_ras = data;

	bt_ras->idle_id = 0;
	ras_notify_ready(bt_ras);
}

bool bt_ras_attach(struct bt_ras *bt_ras, struct bt_gatt_client *client)
{
	bt_uuid_t uuid;

	if (!sessions)
		sessions = queue_new();

	queue_push_tail(sessions, bt_ras);

	if (!client)
		return true;

	if (bt_ras->client)
		return false;

	bt_ras->client = bt_gatt_client_clone(client);
	if (!bt_ras->client)
		return false;

	bt_gatt_client_idle_register(bt_ras->client, ras_idle, bt_ras, NULL);

	bt_uuid16_create(&uuid, RAS_UUID16);

	gatt_db_foreach_service(bt_ras->lrasdb->db, &uuid, foreach_ras_service,
						bt_ras);
	return true;
}

