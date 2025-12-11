// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2025  Qualcomm Incorporated. All rights reserved.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdbool.h>
#include <errno.h>

#include <glib.h>

#include "gdbus/gdbus.h"

#include "bluetooth/bluetooth.h"
#include "bluetooth/sdp.h"
#include "bluetooth/uuid.h"

#include "src/plugin.h"
#include "src/adapter.h"
#include "src/device.h"
#include "src/profile.h"
#include "src/service.h"
#include "src/gatt-database.h"
#include "attrib/gattrib.h"
#include "src/shared/util.h"
#include "src/shared/queue.h"
#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-client.h"
#include "src/shared/ras.h"
#include "attrib/att.h"
#include "src/log.h"

struct ras_data {
	struct btd_device *device;
	struct btd_service *service;
	struct bt_ras *bt_ras;
	unsigned int ready_id;
};

static struct queue *sessions;

static struct ras_data *ras_data_new(struct btd_device *device)
{
	struct ras_data *data;

	data = new0(struct ras_data, 1);
	g_assert(data);
	data->device = device;

	return data;
}

static void ras_data_add(struct ras_data *data)
{
	DBG("%p", data);

	if (queue_find(sessions, NULL, data)) {
		error("data %p already added", data);
		return;
	}

	if (!sessions)
		sessions = queue_new();

	queue_push_tail(sessions, data);

	if (data->service)
		btd_service_set_user_data(data->service, data);
}

static bool match_data(const void *data, const void *match_data)
{
	const struct ras_data *mdata = data;
	const struct bt_ras *bt_ras = match_data;

	return mdata->bt_ras == bt_ras;
}

static void ras_data_free(struct ras_data *data)
{
	if (data->service) {
		btd_service_set_user_data(data->service, NULL);
		bt_ras_set_user_data(data->bt_ras, NULL);
	}

	bt_ras_ready_unregister(data->bt_ras, data->ready_id);
	bt_ras_unref(data->bt_ras);
	free(data);
}

static void ras_data_remove(struct ras_data *data)
{
	DBG("%p", data);

	if (!queue_remove(sessions, data))
		return;

	ras_data_free(data);

	if (queue_isempty(sessions)) {
		queue_destroy(sessions, NULL);
		sessions = NULL;
	}
}

static void ras_detached(struct bt_ras *bt_ras, void *user_data)
{
	struct ras_data *data;

	DBG("%p", bt_ras);

	data = queue_find(sessions, match_data, bt_ras);
	if (!data) {
		error("unable to find session");
		return;
	}

	ras_data_remove(data);
}

static void ras_ready(struct bt_ras *bt_ras, void *user_data)
{
	DBG("bt_ras %p\n", bt_ras);
}

static void ras_attached(struct bt_ras *bt_ras, void *user_data)
{
	struct ras_data *data;
	struct bt_att *att;
	struct btd_device *device;

	DBG("%p", bt_ras);

	data = queue_find(sessions, match_data, bt_ras);
	if (data)
		return;

	att = bt_ras_get_att(bt_ras);
	if (!att)
		return;

	device = btd_adapter_find_device_by_fd(bt_att_get_fd(att));
	if (!device) {
		error("unable to find device");
		return;
	}

	data = ras_data_new(device);
	g_assert(data);
	data->bt_ras = bt_ras;

	ras_data_add(data);
}


static int ras_probe(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct btd_adapter *adapter = device_get_adapter(device);
	struct btd_gatt_database *database = btd_adapter_get_database(adapter);
	struct ras_data *data = btd_service_get_user_data(service);
	char addr[18];

	ba2str(device_get_address(device), addr);
	DBG("%s", addr);

	/*Ignore, if we probed for this device already */
	if (data) {
		error("Profile probed twice for this device");
		return -EINVAL;
	}

	data = ras_data_new(device);
	data->service = service;

	data->bt_ras = bt_ras_new(btd_gatt_database_get_db(database),
				btd_device_get_gatt_db(device));

	if (!data->bt_ras) {
		error("unable to create RAS instance");
		free(data);
		return -EINVAL;
	}

	ras_data_add(data);

	data->ready_id = bt_ras_ready_register(data->bt_ras, ras_ready, service,
								NULL);

	bt_ras_set_user_data(data->bt_ras, service);

	return 0;
}

static void ras_remove(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct ras_data *data;
	char addr[18];

	ba2str(device_get_address(device), addr);
	DBG("%s", addr);

	data = btd_service_get_user_data(service);
	if (!data) {
		error("RAS Service not handled by profile");
		return;
	}

	ras_data_remove(data);
}

static int ras_accept(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	struct bt_gatt_client *client = btd_device_get_gatt_client(device);
	struct ras_data *data = btd_service_get_user_data(service);
	char addr[18];

	ba2str(device_get_address(device), addr);
	DBG("%s", addr);

	if (!data) {
		error("RAS Service not handled by profile");
		return -EINVAL;
	}

	if (!bt_ras_attach(data->bt_ras, client)) {
		error("RAS unable to attach");
		return -EINVAL;
	}

	btd_service_connecting_complete(service, 0);

	return 0;
}

static int ras_disconnect(struct btd_service *service)
{
	DBG(" ");
	btd_service_disconnecting_complete(service, 0);
	return 0;
}

static int ras_connect(struct btd_service *service)
{
	struct btd_device *device = btd_service_get_device(service);
	char addr[18];

	ba2str(device_get_address(device), addr);
	DBG("%s", addr);

	return 0;
}

static int ras_server_probe(struct btd_profile *p,
				  struct btd_adapter *adapter)
{

	struct btd_gatt_database *database = btd_adapter_get_database(adapter);

	DBG("RAS path %s", adapter_get_path(adapter));

	bt_ras_add_db(btd_gatt_database_get_db(database));

	return 0;
}

static void ras_server_remove(struct btd_profile *p,
					struct btd_adapter *adapter)
{
	DBG("RAS server remove");
}
/* Profile definition */
static struct btd_profile ras_profile = {
	.name		= "ras",
	.priority	= BTD_PROFILE_PRIORITY_MEDIUM,
	.remote_uuid	= GATT_UUID,
	.local_uuid	= RAS_UUID,

	.device_probe	= ras_probe,
	.device_remove	= ras_remove,

	.accept		= ras_accept,
	.connect	= ras_connect,
	.disconnect	= ras_disconnect,

	.adapter_probe = ras_server_probe,
	.adapter_remove = ras_server_remove,

	.experimental	= true,
};

static unsigned int ras_id = 0;
/* Plugin init/exit */
static int ras_init(void)
{
	DBG("");
	if (!(g_dbus_get_flags() & G_DBUS_FLAG_ENABLE_EXPERIMENTAL)) {
		DBG("D-Bus experimental not enabled");
		return -ENOTSUP;
	}

	btd_profile_register(&ras_profile);
	ras_id = bt_ras_register(ras_attached, ras_detached, NULL);

	return 0;
}

static void ras_exit(void)
{
	if (g_dbus_get_flags() & G_DBUS_FLAG_ENABLE_EXPERIMENTAL) {
		btd_profile_unregister(&ras_profile);
		bt_ras_unregister(ras_id);
	}
}

/* Plugin definition */
BLUETOOTH_PLUGIN_DEFINE(ras, VERSION, BLUETOOTH_PLUGIN_PRIORITY_DEFAULT,
			ras_init, ras_exit)

