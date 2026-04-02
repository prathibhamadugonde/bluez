// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * BlueZ - Bluetooth protocol stack for Linux
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
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
#include "src/shared/att.h"
#include "src/shared/gatt-db.h"
#include "src/shared/gatt-server.h"
#include "src/shared/gatt-client.h"
#include "src/shared/rap.h"

#define DBG(_rap, fmt, arg...) \
	rap_debug(_rap, "%s:%s() " fmt, __FILE__, __func__, ## arg)

#define RAS_UUID16			0x185B

/* Total number of attribute handles reserved for the RAS service */
#define RAS_TOTAL_NUM_HANDLES		18

#define RAS_RANGING_HEADER_SIZE 4 /* 2(rc+cfg) + 1(tx_pwr) + 1(4 bits antenna_mask, 2 bits reserved, 2 bits pct_format) */
#define TOTAL_RAS_RANGING_HEADER_SIZE 5
#define ATT_OVERHEAD 3 /* 1(opcode) + 2(char handle) */
#define RAS_STEP_ABORTED_BIT   0x80/* set step aborted */
#define RAS_SUBEVENT_HEADER_SIZE 8
#define EXTRA_LOGGING_DUMPING_RAW_BYTES
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
static void dbg_hexdump_line(struct bt_rap *rap, const char *prefix, const uint8_t *buf, size_t len, size_t max);
static void dbg_decode_ranging_header(struct bt_rap *rap, const uint8_t *buf, size_t len);
static void dbg_decode_ras_subevent_header(struct bt_rap *rap, const uint8_t *buf, size_t len);
#endif

enum pct_format {
    IQ = 0,
    PHASE = 1,
};

enum ranging_done_status {
    RANGING_DONE_ALL_RESULTS_COMPLETE = 0x0,
    RANGING_DONE_PARTIAL_RESULTS = 0x1,
    RANGING_DONE_ABORTED = 0xF,
};

enum subevent_done_status {
    SUBEVENT_DONE_ALL_RESULTS_COMPLETE = 0x0,
    SUBEVENT_DONE_PARTIAL_RESULTS = 0x1,
    SUBEVENT_DONE_ABORTED = 0xF,
};

enum ranging_abort_reason {
    RANGING_ABORT_NO_ABORT = 0x0,
    RANGING_ABORT_LOCAL_HOST_OR_REMOTE = 0x1,
    RANGING_ABORT_INSUFFICIENT_FILTERED_CHANNELS = 0x2,
    RANGING_ABORT_INSTANT_HAS_PASSED = 0x3,
    RANGING_ABORT_UNSPECIFIED = 0xF,
};

enum subevent_abort_reason {
    SUBEVENT_ABORT_NO_ABORT = 0x0,
    SUBEVENT_ABORT_LOCAL_HOST_OR_REMOTE = 0x1,
    SUBEVENT_ABORT_NO_CS_SYNC_RECEIVED = 0x2,
    SUBEVENT_ABORT_SCHEDULING_CONFLICTS_OR_LIMITED_RESOURCES = 0x3,
    SUBEVENT_ABORT_UNSPECIFIED = 0xF,
};

struct segmentation_header {
    uint8_t first_segment : 1;
    uint8_t last_segment : 1;
    uint8_t rolling_segment_counter : 6;
};

struct ranging_header {
    uint16_t ranging_counter : 12;
    uint8_t configuration_id : 4;
    int8_t selected_tx_power;
    uint8_t antenna_paths_mask : 4;
    uint8_t _reserved_ : 2;
    uint8_t pct_format : 2;
}__packed;

struct ras_subevent_header {
    uint16_t start_acl_conn_event;
    uint16_t frequency_compensation;
    uint8_t ranging_done_status;
    uint8_t subevent_done_status;
    uint8_t ranging_abort_reason;
    uint8_t subevent_abort_reason;
    int8_t reference_power_level;
    uint8_t num_steps_reported;
};

struct ras_subevent {
    struct ras_subevent_header subevent_header;
    uint8_t subevent_data[0];
};

/* Role maps to Core CS roles (initiator/reflector) */
typedef enum {
    CS_ROLE_INITIATOR = 0x00,
    CS_ROLE_REFLECTOR = 0x01,
} cs_role_t;

#define CS_INVALID_CONFIG_ID   0xFF
/* Minimal enums (align to controller values if needed) */
typedef enum {
  CS_PROC_ALL_RESULTS_COMPLETE = 0x00,
  CS_PROC_PARTIAL_RESULTS      = 0x01,
  CS_PROC_ABORTED              = 0x02
} CsProcedureDoneStatus;
/* Generic dynamic array */
typedef struct {
  void*   data;
  size_t  len;
  size_t  cap;
  size_t  elem_size;
} DynArray;

/* Basic array helpers */
void da_init(DynArray* da, size_t elem_size);
void da_clear(DynArray* da);
void da_free(DynArray* da);
bool da_append_bytes(DynArray* da, const uint8_t* bytes, size_t len);

/* Main CsProcedureData  */
typedef struct {
  /* Identity and counters */
  uint16_t counter;
  uint8_t  num_antenna_paths;
  /* Flags and status */
  CsProcedureDoneStatus local_status;
  CsProcedureDoneStatus remote_status;
  bool contains_complete_subevent_;
  /* RAS aggregation */
  struct segmentation_header segmentation_header_;
  struct ranging_header      ranging_header_;
  DynArray           ras_raw_data_;        /* uint8_t raw concatenated bytes */
  uint16_t           ras_raw_data_index_;
  struct ras_subevent_header  ras_subevent_header_;
  DynArray           ras_subevent_data_;   /* uint8_t buffer per subevent */
  uint8_t            ras_subevent_counter_;
  /* Reference power levels */
  int8_t initiator_reference_power_level;
  int8_t reflector_reference_power_level;
  bool ranging_header_prepended_;
  bool ras_subevent_header_emitted;
} CsProcedureData;

struct cstracker {
    cs_role_t           role;                 /* INITIATOR or REFLECTOR */
    uint8_t             config_id;            /* kInvalidConfigId if unknown */
    uint8_t             selected_tx_power;    /* from PROC_ENABLE_COMPLETE */
    CsProcedureData *current_proc;
};

/* Ranging Service */
struct ras {
	struct bt_rap_db *rapdb;

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

struct bt_rap_db {
	struct gatt_db *db;
	struct ras *ras;
};

struct bt_rap {
	int ref_count;
	struct bt_rap_db *lrapdb;
	struct bt_rap_db *rrapdb;
	struct bt_gatt_client *client;
    struct bt_gatt_server *server;
	struct bt_att *att;

	unsigned int idle_id;

	struct queue *notify;
	struct queue *pending;
	struct queue *ready_cbs;

	bt_rap_debug_func_t debug_func;
	bt_rap_destroy_func_t debug_destroy;
	void *debug_data;
	void *user_data;
    struct cstracker *resptracker;
};

static struct queue *rap_db;
static struct queue *bt_rap_cbs;
static struct queue *sessions;

struct bt_rap_cb {
	unsigned int id;
	bt_rap_func_t attached;
	bt_rap_func_t detached;
	void *user_data;
};

struct bt_rap_ready {
	unsigned int id;
	bt_rap_ready_func_t func;
	bt_rap_destroy_func_t destroy;
	void *data;
};

static bool real_time_enabled = false;
static bool on_demand_enabled = false;
struct gatt_db_attribute *global_real_time_char = NULL;
struct gatt_db_attribute *global_on_demand_char = NULL;

static uint16_t g_last_proc_counter = 0;
static uint16_t g_last_start_acl_conn_evt_counter = 0;
static uint16_t g_last_freq_comp = 0;
static int8_t  g_last_ref_pwr_lvl = 0;
uint16_t DefaultRasMtu = 517;
uint8_t RasSegmentHeaderSize = 1;
/* Cs procedure data helper functions */
/* ---- DynArray implementation ---- */
static bool da_grow(DynArray* da, size_t min_cap)
{
  if (da->cap >= min_cap) return true;
  size_t new_cap = da->cap ? da->cap * 2 : 8;
  // Check multiplication overflow before loop
  if (da->elem_size > SIZE_MAX / new_cap) {
    return false;
  }
  while (new_cap < min_cap) {
    if (new_cap > SIZE_MAX / 2) {
      return false;
    }
    new_cap *= 2;
    // Re-check multiplication overflow after doubling
    if (da->elem_size > SIZE_MAX / new_cap) {
      return false;
    }
  }
  void* p = realloc(da->data, new_cap * da->elem_size);
  if (!p) return false;
  da->data = p;
  da->cap = new_cap;
  return true;
}
void da_init(DynArray* da, size_t elem_size)
{
  da->data = NULL; da->len = 0; da->cap = 0; da->elem_size = elem_size;
}
void da_clear(DynArray* da) { da->len = 0; }
void da_free(DynArray* da)
{
  if (!da) return;
  free(da->data); da->data = NULL; da->len = 0; da->cap = 0; da->elem_size = 0;
}
bool da_append_bytes(DynArray* da, const uint8_t* bytes, size_t len)
{
  if (!da || !bytes || da->elem_size != 1) return false;
  if (!da_grow(da, da->len + len)) return false;
  memcpy((uint8_t*)da->data + da->len, bytes, len);
  da->len += len;
  return true;
}
/* ---- Utility ---- */
CsProcedureData* cs_procedure_data_create(uint16_t procedure_counter,
                                          uint8_t num_antenna_paths,
                                          uint8_t configuration_id,
                                          uint8_t selected_tx_power)
{
  CsProcedureData* d = (CsProcedureData*)calloc(1, sizeof(CsProcedureData));
  if (!d) return NULL;
  d->counter = procedure_counter;
  d->num_antenna_paths = num_antenna_paths;
  /* Status defaults */
  d->local_status  = CS_PROC_PARTIAL_RESULTS;
  d->remote_status = CS_PROC_PARTIAL_RESULTS;
  d->contains_complete_subevent_ = false;
  /* RAS headers/init */
  d->segmentation_header_.first_segment = 1;
  d->segmentation_header_.last_segment = 0;
  d->segmentation_header_.rolling_segment_counter = 0;
  d->ranging_header_.ranging_counter   = procedure_counter;
  d->ranging_header_.configuration_id  = configuration_id;
  d->ranging_header_.selected_tx_power = selected_tx_power;
  /* Compute antenna paths mask inline */
  d->ranging_header_.antenna_paths_mask = 0;
  for (uint8_t i = 0; i < num_antenna_paths; i++)
    d->ranging_header_.antenna_paths_mask |= (1u << i);
  d->ranging_header_.pct_format = IQ;
  da_init(&d->ras_raw_data_,      1);
  d->ras_raw_data_index_ = 0;
  da_init(&d->ras_subevent_data_, 1);
  d->ras_subevent_counter_ = 0;
  d->initiator_reference_power_level = 0;
  d->reflector_reference_power_level = 0;
  d->ranging_header_prepended_ = false;
  d->ras_subevent_header_emitted = false;
  return d;
}
void cs_procedure_data_destroy(CsProcedureData* d)
{
  if (!d) return;
  da_free(&d->ras_raw_data_);
  da_free(&d->ras_subevent_data_);
  free(d);
}
/* ---- Status/flags ---- */
void cs_pd_set_local_status(CsProcedureData* d, CsProcedureDoneStatus s) { if (d) d->local_status = s; }
void cs_pd_set_remote_status(CsProcedureData* d, CsProcedureDoneStatus s) { if (d) d->remote_status = s; }
void cs_pd_set_reference_power_levels(CsProcedureData* d, int8_t init_lvl, int8_t ref_lvl)
{
  if (!d) return; d->initiator_reference_power_level = init_lvl; d->reflector_reference_power_level = ref_lvl;
}
/* ---- RAS helpers ---- */
void cs_pd_ras_begin_subevent(CsProcedureData* d,
                              uint16_t start_acl_conn_event,
                              uint16_t frequency_compensation,
                              int8_t reference_power_level)
{
  if (!d) return;
  d->ras_subevent_counter_++;
  d->ras_subevent_header_.start_acl_conn_event = start_acl_conn_event;
  d->ras_subevent_header_.frequency_compensation = frequency_compensation;
  d->ras_subevent_header_.reference_power_level = reference_power_level;
  d->ras_subevent_header_.num_steps_reported = 0;
  d->ras_subevent_header_emitted = false;
  da_clear(&d->ras_subevent_data_);
}
bool cs_pd_ras_append_subevent_bytes(CsProcedureData* d, const uint8_t* bytes, size_t len)
{
  if (!d || !bytes || len == 0) return false;
  return da_append_bytes(&d->ras_subevent_data_, bytes, len);
}
static inline size_t serialize_ras_subevent_header(const struct ras_subevent_header *h,
                                                   uint8_t *out, size_t out_len)
{
    if (!h || !out || out_len < RAS_SUBEVENT_HEADER_SIZE) return 0;
    /* start_acl_conn_event: 16-bit little endian
     * If your start ACL connection event is only tracked as 8-bit in begin_subevent,
     * we still write a 16-bit LE value with high byte = 0 for now.
     */
    uint16_t start_le = (uint16_t)h->start_acl_conn_event; /* widen if stored as uint8_t */
    out[0] = (uint8_t)(start_le & 0xFF);
    out[1] = (uint8_t)((start_le >> 8) & 0xFF);
    /* frequency_compensation: 16-bit little endian */
    uint16_t freq_le = (uint16_t)h->frequency_compensation;
    out[2] = (uint8_t)(freq_le & 0xFF);
    out[3] = (uint8_t)((freq_le >> 8) & 0xFF);
    /* Done statuses (4+4 bits): [ranging_done | subevent_done << 4] */
    out[4] = (uint8_t)((h->ranging_done_status & 0x0F) |
                       ((h->subevent_done_status & 0x0F) << 4));
    /* Abort reasons (4+4 bits): [ranging_abort | subevent_abort << 4] */
    out[5] = (uint8_t)((h->ranging_abort_reason & 0x0F) |
                       ((h->subevent_abort_reason & 0x0F) << 4));
    /* Reference power level (8-bit) */
    out[6] = (uint8_t)h->reference_power_level;
    /* Num steps reported (8-bit) */
    out[7] = (uint8_t)h->num_steps_reported;
    return RAS_SUBEVENT_HEADER_SIZE;
}

bool cs_pd_ras_commit_subevent(CsProcedureData* d,
                               uint8_t num_steps_reported,
                               uint8_t ranging_done_status,
                               uint8_t subevent_done_status,
                               uint8_t ranging_abort_reason,
                               uint8_t subevent_abort_reason)
{
  if (!d) return false;
  /* Update header fields */
  d->ras_subevent_header_.num_steps_reported =
	  (uint8_t)(d->ras_subevent_header_.num_steps_reported + num_steps_reported);
  d->ras_subevent_header_.ranging_done_status   = ranging_done_status;
  d->ras_subevent_header_.subevent_done_status  = subevent_done_status;
  d->ras_subevent_header_.ranging_abort_reason  = ranging_abort_reason;
  d->ras_subevent_header_.subevent_abort_reason = subevent_abort_reason;
  if (subevent_done_status == SUBEVENT_DONE_ALL_RESULTS_COMPLETE) {
    d->contains_complete_subevent_ = true;
  }
  /* Still partial? Only update header & keep ras_subevent_data_ growing. */
  if (subevent_done_status == SUBEVENT_DONE_PARTIAL_RESULTS)
    return true;
  /* Final chunk (ALL_RESULTS_COMPLETE or ABORTED) – now emit header+all steps */
  if (!d->ras_subevent_header_emitted) {
    size_t hdr_sz = RAS_SUBEVENT_HEADER_SIZE;
    size_t payload_sz = d->ras_subevent_data_.len;
    size_t total = hdr_sz + payload_sz;
  uint8_t *buf = (uint8_t*)malloc(total);
  if (!buf) return false;
  size_t w = serialize_ras_subevent_header(&d->ras_subevent_header_, buf, total);
  if (w != hdr_sz) {
    free(buf);
    return false;
  }
  if (payload_sz > 0) {
    memcpy(buf + hdr_sz, (const uint8_t*)d->ras_subevent_data_.data, payload_sz);
  }
  /* Append to raw data buffer (like append_vector) */
  bool ok = da_append_bytes(&d->ras_raw_data_, buf, total);
  free(buf);
  if (!ok) return false;
    da_clear(&d->ras_subevent_data_);
    d->ras_subevent_header_emitted = true;
  }
  return true;
}
static struct ras *rap_get_ras(struct bt_rap *rap)
{
	if (!rap)
		return NULL;

	if (rap->rrapdb->ras)
		return rap->rrapdb->ras;

	rap->rrapdb->ras = new0(struct ras, 1);
	rap->rrapdb->ras->rapdb = rap->rrapdb;

	return rap->rrapdb->ras;
}

static struct bt_gatt_server *btrap_get_gattserver(struct bt_rap *rap)
{
	if (!rap)
		return NULL;
	if (rap->server)
		return rap->server;
	return NULL;
}

static void rap_detached(void *data, void *user_data)
{
	struct bt_rap_cb *cb = data;
	struct bt_rap *rap = user_data;

	cb->detached(rap, cb->user_data);
}

void bt_rap_detach(struct bt_rap *rap)
{
	if (!queue_remove(sessions, rap))
		return;

	bt_gatt_client_idle_unregister(rap->client, rap->idle_id);
	bt_gatt_client_unref(rap->client);
	rap->client = NULL;

	queue_foreach(bt_rap_cbs, rap_detached, rap);
}

static void rap_db_free(void *data)
{
	struct bt_rap_db *rapdb = data;

	if (!rapdb)
		return;

	gatt_db_unref(rapdb->db);

	free(rapdb->ras);
	free(rapdb);
}

static void rap_ready_free(void *data)
{
	struct bt_rap_ready *ready = data;

	if (ready->destroy)
		ready->destroy(ready->data);

	free(ready);
}

static void rap_free(void *data)
{
	struct bt_rap *rap = data;

	bt_rap_detach(rap);

	rap_db_free(rap->rrapdb);

	if (rap->resptracker) {
		free(rap->resptracker);
		rap->resptracker = NULL;
	}
	
	queue_destroy(rap->notify, free);
	queue_destroy(rap->pending, NULL);
	queue_destroy(rap->ready_cbs, rap_ready_free);

	free(rap);
}

bool bt_rap_set_user_data(struct bt_rap *rap, void *user_data)
{
	if (!rap)
		return false;

	rap->user_data = user_data;

	return true;
}

static bool rap_db_match(const void *data, const void *match_data)
{
	const struct bt_rap_db *rapdb = data;
	const struct gatt_db *db = match_data;

	return rapdb->db == db;
}

struct bt_att *bt_rap_get_att(struct bt_rap *rap)
{
	if (!rap)
		return NULL;

	if (rap->att)
		return rap->att;

	return bt_gatt_client_get_att(rap->client);
}

struct bt_rap *bt_rap_ref(struct bt_rap *rap)
{
	if (!rap)
		return NULL;

	__sync_fetch_and_add(&rap->ref_count, 1);

	return rap;
}

void bt_rap_unref(struct bt_rap *rap)
{
	if (!rap)
		return;

	if (__sync_sub_and_fetch(&rap->ref_count, 1))
		return;

	rap_free(rap);
}

static void rap_debug(struct bt_rap *rap, const char *format, ...)
{
	va_list ap;

	if (!rap || !format || !rap->debug_func)
		return;

	va_start(ap, format);
	util_debug_va(rap->debug_func, rap->debug_data, format, ap);
	va_end(ap);
}

bool bt_rap_set_debug(struct bt_rap *rap, bt_rap_debug_func_t func,
			void *user_data, bt_rap_destroy_func_t destroy)
{
	if (!rap)
		return false;

	if (rap->debug_destroy)
		rap->debug_destroy(rap->debug_data);

	rap->debug_func = func;
	rap->debug_destroy = destroy;
	rap->debug_data = user_data;

	return true;
}

static void cs_tracker_init(struct cstracker *t, uint16_t conn_handle)
{
    if (!t)
        return;
    memset(t, 0, sizeof(*t));
    t->role = CS_ROLE_REFLECTOR;
    t->config_id = CS_INVALID_CONFIG_ID;
    t->selected_tx_power = 0;
}

static void ras_features_read_cb(struct gatt_db_attribute *attrib,
				 unsigned int id, uint16_t offset,
				 uint8_t opcode, struct bt_att *att,
				 void *user_data)
{
	/*
	 * Feature mask: bits 0-2 set:
	 *  - Real-time ranging
	 *  - Retrieve stored results
	 *  - Abort operation
	 */
	uint8_t value[4] = { 0x01, 0x00, 0x00, 0x00 };

	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void ras_ondemand_read_cb(struct gatt_db_attribute *attrib,
				 unsigned int id, uint16_t offset,
				 uint8_t opcode, struct bt_att *att,
				 void *user_data)
{
	/* No static read data – on‑demand data is pushed via
	 * notifications
	 */
	gatt_db_attribute_read_result(attrib, id, 0, NULL, 0);
}

/*
 * Control point handler.
 * Parses the opcode and acts on queued data (implementation TBD).
 */
static void ras_control_point_write_cb(struct gatt_db_attribute *attrib,
				       unsigned int id, uint16_t offset,
				       const uint8_t *value, size_t len,
				       uint8_t opcode, struct bt_att *att,
				       void *user_data)
{
	/* Control point handler - implementation TBD */
}

/* Data Ready – returns the latest ranging counter. */
static void ras_data_ready_read_cb(struct gatt_db_attribute *attrib,
				   unsigned int id, uint16_t offset,
				   uint8_t opcode, struct bt_att *att,
				   void *user_data)
{
	uint16_t counter = 0;
	uint8_t value[2];

	put_le16(counter, value);
	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

/* Data Overwritten – indicates how many results were overwritten. */
static void ras_data_overwritten_read_cb(struct gatt_db_attribute *attrib,
					 unsigned int id, uint16_t offset,
					 uint8_t opcode, struct bt_att *att,
					 void *user_data)
{
	uint8_t value[2] = { 0x00, 0x00 };

	gatt_db_attribute_read_result(attrib, id, 0, value, sizeof(value));
}

static void ras_ccc_write_cb(struct gatt_db_attribute *attrib,
                             unsigned int id, uint16_t offset,
                             const uint8_t *value, size_t len,
                             uint8_t opcode , struct bt_att *att,
                             void *user_data)
{
	struct ras *ras = user_data;

	uint16_t handle = gatt_db_attribute_get_handle(attrib);
    uint16_t realtime_char_handle = ras->realtime_chrc ? gatt_db_attribute_get_handle(ras->realtime_chrc) : 0;
    uint16_t ondemand_char_handle = ras->ondemand_chrc ? gatt_db_attribute_get_handle(ras->ondemand_chrc) : 0;
    bool notify_enabled = (value[0] & 0x01);
    bool indicate_enabled = (value[0] & 0x02);
    /* Update global pointers */
    global_real_time_char = ras->realtime_chrc;
    global_on_demand_char = ras->ondemand_chrc;

    if (handle == (realtime_char_handle + 1)) {
        /* Real-time CCC */
        if (notify_enabled || indicate_enabled) {
            if (on_demand_enabled) {
                gatt_db_attribute_write_result(attrib, id, 0xFD);
                return;
            }
            real_time_enabled = true;
        } else {
            real_time_enabled = false;
        }
    } else if (handle == (ondemand_char_handle + 1)) {
        /* On-demand CCC */
        if (notify_enabled || indicate_enabled) {
            if (real_time_enabled) {
                gatt_db_attribute_write_result(attrib, id, 0xFD);
                return;
            }
            on_demand_enabled = true;
            /* No timer needed for on-demand */
        } else {
            on_demand_enabled = false;
        }
    } else {
        gatt_db_attribute_write_result(attrib, id, 0);
        return;
    }
    gatt_db_attribute_write_result(attrib, id, 0);
}

/* Service registration – store attribute pointers */
static struct ras *register_ras_service(struct gatt_db *db)
{
	struct ras *ras;
	struct gatt_db_attribute *service;
	bt_uuid_t uuid;

	if (!db)
		return NULL;

	ras = new0(struct ras, 1);
	if (!ras)
		return NULL;
		
	gatt_db_ccc_register(db, NULL, ras_ccc_write_cb, NULL, ras);
	
	/* Primary RAS service */
	bt_uuid16_create(&uuid, RAS_UUID16);
	service = gatt_db_add_service(db, &uuid, true, RAS_TOTAL_NUM_HANDLES);
	if (!service) {
		free(ras);
		return NULL;
	}

	ras->svc = service;

	/* RAS Features */
	bt_uuid16_create(&uuid, RAS_FEATURES_UUID);
		ras->feat_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_READ |
						  BT_ATT_PERM_READ_ENCRYPT,
						  BT_GATT_CHRC_PROP_READ,
						  ras_features_read_cb,
						  NULL, ras);

	/* Real-time Ranging Data */
	bt_uuid16_create(&uuid, RAS_REALTIME_DATA_UUID);
	ras->realtime_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_READ |
						  BT_ATT_PERM_READ_ENCRYPT,
						  BT_GATT_CHRC_PROP_NOTIFY |
						  BT_GATT_CHRC_PROP_INDICATE,
						  NULL, NULL, ras);

	ras->realtime_chrc_ccc =
		gatt_db_service_add_ccc(ras->svc,
					BT_ATT_PERM_READ |
					BT_ATT_PERM_WRITE);

	/* On-demand Ranging Data */
	bt_uuid16_create(&uuid, RAS_ONDEMAND_DATA_UUID);
	ras->ondemand_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_READ |
						  BT_ATT_PERM_READ_ENCRYPT,
						  BT_GATT_CHRC_PROP_NOTIFY |
						  BT_GATT_CHRC_PROP_INDICATE,
						  ras_ondemand_read_cb, NULL,
						  ras);

	gatt_db_service_add_ccc(ras->svc,
				BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

	/* RAS Control Point */
	bt_uuid16_create(&uuid, RAS_CONTROL_POINT_UUID);
	ras->cp_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_WRITE |
						  BT_ATT_PERM_WRITE_ENCRYPT,
				BT_GATT_CHRC_PROP_WRITE_WITHOUT_RESP |
						  BT_GATT_CHRC_PROP_INDICATE,
						  NULL,
						  ras_control_point_write_cb,
						  ras);

	gatt_db_service_add_ccc(ras->svc,
				BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

	/* RAS Data Ready */
	bt_uuid16_create(&uuid, RAS_DATA_READY_UUID);
	ras->ready_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_READ |
						  BT_ATT_PERM_READ_ENCRYPT,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY |
						  BT_GATT_CHRC_PROP_INDICATE,
						  ras_data_ready_read_cb, NULL,
						  ras);

	gatt_db_service_add_ccc(ras->svc,
				BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

	/* RAS Data Overwritten */
	bt_uuid16_create(&uuid, RAS_DATA_OVERWRITTEN_UUID);
	ras->overwritten_chrc =
		gatt_db_service_add_characteristic(ras->svc, &uuid,
						  BT_ATT_PERM_READ |
						  BT_ATT_PERM_READ_ENCRYPT,
						  BT_GATT_CHRC_PROP_READ |
						  BT_GATT_CHRC_PROP_NOTIFY |
						  BT_GATT_CHRC_PROP_INDICATE,
						  ras_data_overwritten_read_cb,
						  NULL, ras);

	gatt_db_service_add_ccc(ras->svc,
				BT_ATT_PERM_READ | BT_ATT_PERM_WRITE);

	/* Activate the service */
	gatt_db_service_set_active(ras->svc, true);

	return ras;
}

static struct bt_rap_db *rap_db_new(struct gatt_db *db)
{
	struct bt_rap_db *rapdb;

	if (!db)
		return NULL;

	rapdb = new0(struct bt_rap_db, 1);
	if (!rapdb)
		return NULL;

	rapdb->db = gatt_db_ref(db);

	if (!rap_db)
		rap_db = queue_new();

	rapdb->ras = register_ras_service(db);
	if (rapdb->ras)
		rapdb->ras->rapdb = rapdb;

	queue_push_tail(rap_db, rapdb);

	return rapdb;
}

static struct bt_rap_db *rap_get_db(struct gatt_db *db)
{
	struct bt_rap_db *rapdb;

	rapdb = queue_find(rap_db, rap_db_match, db);
	if (rapdb)
		return rapdb;

	return rap_db_new(db);
}

void bt_rap_add_db(struct gatt_db *db)
{
	rap_db_new(db);
}

unsigned int bt_rap_register(bt_rap_func_t attached, bt_rap_func_t detached,
			     void *user_data)
{
	struct bt_rap_cb *cb;
	static unsigned int id;

	if (!attached && !detached)
		return 0;

	if (!bt_rap_cbs)
		bt_rap_cbs = queue_new();

	cb = new0(struct bt_rap_cb, 1);
	cb->id = ++id ? id : ++id;
	cb->attached = attached;
	cb->detached = detached;
	cb->user_data = user_data;

	queue_push_tail(bt_rap_cbs, cb);

	return cb->id;
}

static bool match_id(const void *data, const void *match_data)
{
	const struct bt_rap_cb *cb = data;
	unsigned int id = PTR_TO_UINT(match_data);

	return cb->id == id;
}

bool bt_rap_unregister(unsigned int id)
{
	struct bt_rap_cb *cb;

	cb = queue_remove_if(bt_rap_cbs, match_id, UINT_TO_PTR(id));
	if (!cb)
		return false;

	free(cb);

	return true;
}

bool bt_rap_gatt_server_attach(struct bt_rap *rap, struct bt_gatt_server *server)
{
    if (!rap)
        return false;

    if (!server)
        return false;

    rap->server = server;

    if (!rap->server)
        return false;

    return true;
}

static inline size_t serialize_segmentation_header(const struct segmentation_header *s,
                                                   uint8_t *out, size_t out_len) {
  if (!s || !out || out_len < 1) return 0;
  /* [0] bit0: first, bit1: last, bits2..7: rolling counter */
  out[0] = (s->first_segment ? 0x01 : 0x00) |
           (s->last_segment ? 0x02 : 0x00) |
           ((s->rolling_segment_counter & 0x3F) << 2);
  return 1;
}

static inline size_t serialize_ranging_header(const struct ranging_header *r, uint8_t *out, size_t out_len)
{
  if (!r || !out || out_len < RAS_RANGING_HEADER_SIZE) return 0;
  /* Little-endian pack: [rc (12 bits) | cfg_id (4 bits)] into 2 bytes */
  uint16_t rcid = (uint16_t)((r->ranging_counter & 0x0FFF) |
                             ((r->configuration_id & 0x0F) << 12));
  put_le16(rcid, out + 0);                           /* bytes 0..1 */
  out[2] = (uint8_t)r->selected_tx_power;            /* byte 2 */
  /* Byte 3: antenna_paths_mask (bits 0..3) | _reserved_ (bits 4..5) | pct_format (bits 6..7) */
  uint8_t mask   = (uint8_t)(r->antenna_paths_mask & 0x0F);
  uint8_t res    = (uint8_t)((r->_reserved_   & 0x03) << 4);
  uint8_t pct    = (uint8_t)((r->pct_format   & 0x03) << 6);
  out[3] = (uint8_t)(mask | res | pct);
  return RAS_RANGING_HEADER_SIZE;
}
static inline uint16_t ras_att_value_payload_max(struct bt_rap *rap)
{
  struct bt_att *att = bt_rap_get_att(rap);
  uint16_t mtu = att ? bt_att_get_mtu(att) : DefaultRasMtu; // fallback
  return (uint16_t)(mtu > ATT_OVERHEAD ? (mtu - ATT_OVERHEAD - TOTAL_RAS_RANGING_HEADER_SIZE - RasSegmentHeaderSize) : 0);
}
/* Prepend bytes to a DynArray of elem_size=1 */
static bool da_prepend_bytes(DynArray* da, const uint8_t* bytes, size_t len)
{
  if (!da || !bytes || len == 0 || da->elem_size != 1) return false;
  if (!da_grow(da, da->len + len)) return false;      /* grows capacity, keeps len */
  /* make room at front */
  memmove((uint8_t*)da->data + len, (uint8_t*)da->data, da->len);
  memcpy((uint8_t*)da->data, bytes, len);
  da->len += len;
  return true;
}
/* Append the 4-byte RangingHeader to ras_raw_data_ on first segment only */
static bool ras_maybe_prepend_ranging_header(CsProcedureData* d)
{
  if (!d) return false;
  if (d->ranging_header_prepended_)   return false;   /* already done, hard guard */
  if (!d->segmentation_header_.first_segment) return false;   /* already sent a segment */
  if (d->ras_raw_data_index_ != 0) return false;              /* already consumed some bytes */
  uint8_t hdr[RAS_RANGING_HEADER_SIZE];
  size_t w = serialize_ranging_header(&d->ranging_header_, hdr, sizeof(hdr));
  if (w != RAS_RANGING_HEADER_SIZE) return false;
  bool ok = da_prepend_bytes(&d->ras_raw_data_, hdr, w);
  if (ok)
	  d->ranging_header_prepended_ = true;
  return ok;
}
static struct gatt_db_attribute *select_ras_out_char(void)
{
  if (on_demand_enabled && global_on_demand_char) return global_on_demand_char;
  if (real_time_enabled && global_real_time_char) return global_real_time_char;
  return NULL;
}
static void send_ras_segment_data(struct bt_rap *rap, CsProcedureData *proc)
{
  if (!rap || !proc) return;
  DBG(rap,"came");
  struct bt_gatt_server *server = btrap_get_gattserver(rap);
  if (!server) return;
  struct gatt_db_attribute *out_char = select_ras_out_char();
  if (!out_char) {
    DBG(rap,"No CCC-enabled RAS characteristic");
    return;
  }
  uint16_t handle = gatt_db_attribute_get_handle(out_char);
  uint16_t value_max = ras_att_value_payload_max(rap);
  if (value_max == 0) {
    DBG(rap,"value_max=0 (MTU not available?)");
    return;
  }
  const uint16_t header_len = RasSegmentHeaderSize;
  if (value_max <= header_len) {
    DBG(rap,"value_max(%u) too small for header", value_max);
    return;
  }
  const uint16_t raw_payload_size = (uint16_t)(value_max - header_len);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
  /* If this is the first segment and we have the RangingHeader at index 0, decode it */
  if (proc->segmentation_header_.first_segment &&
      proc->ras_raw_data_.len >= RAS_RANGING_HEADER_SIZE &&
      proc->ras_raw_data_index_ == 0) {
      const uint8_t *rhdr = (const uint8_t*)proc->ras_raw_data_.data;
      dbg_decode_ranging_header(rap, rhdr, RAS_RANGING_HEADER_SIZE);
  }
 /* Debug: dump the raw RAS data buffer before segmenting */
  dbg_hexdump_line(rap, "RAS raw data before seg",
                   (const uint8_t*)proc->ras_raw_data_.data,
                   proc->ras_raw_data_.len,
                   64);
#endif
  // Convert tail recursion to loop
  while (true) {
    size_t total_len = proc->ras_raw_data_.len;
    size_t index = proc->ras_raw_data_index_;
    if (index > total_len) index = total_len;
    size_t unsent_data_size = total_len - index;
    if (unsent_data_size == 0) return;
    /* Set last_segment if:
     * 1. The entire procedure is complete (not partial), OR
     * 2. We have a complete subevent and all data fits in one segment
     */
    if ((proc->local_status != CS_PROC_PARTIAL_RESULTS && unsent_data_size <= raw_payload_size) ||
        (proc->contains_complete_subevent_ && unsent_data_size <= raw_payload_size)) {
      proc->segmentation_header_.last_segment = 1;
    } else {
      proc->segmentation_header_.last_segment = 0;
    }
    /* Only wait for more data if we don't have enough AND it's not the last segment */
    if (unsent_data_size < raw_payload_size && proc->segmentation_header_.last_segment == 0) {
      DBG(rap,"send_ras_segment_data: waiting for more data (unsent=%zu < payload=%u)",
          unsent_data_size, raw_payload_size);
      return;
    }
    uint16_t copy_size = (uint16_t)((unsent_data_size < raw_payload_size) ? unsent_data_size : raw_payload_size);
    uint16_t seg_len = (uint16_t)(header_len + copy_size);
    uint8_t *seg = (uint8_t *)malloc(seg_len);
    if (!seg) {
      DBG(rap,"send_ras_segment_data: OOM (%u)", seg_len);
      return;
    }
    uint16_t wr = 0;
    wr += (uint16_t)serialize_segmentation_header(&proc->segmentation_header_, seg + wr, seg_len - wr);
    memcpy(seg + wr, (const uint8_t *)proc->ras_raw_data_.data + index, copy_size);
    wr += copy_size;
    DBG(rap,"RAS seg: ch=0x%04x first=%u last=%u rsc=%u body=%u total=%u",
		    handle,
		    proc->segmentation_header_.first_segment,
		    proc->segmentation_header_.last_segment,
		    (unsigned)(proc->segmentation_header_.rolling_segment_counter & 0x3F),
		    (unsigned)copy_size, (unsigned)wr);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
    /* Print hex dump of the segment before sending */
    dbg_hexdump_line(rap, "RAS seg hex", seg, wr, 1000);
    /* Optional: decode first subevent header visible in this segment body */
    const uint8_t *body = seg + header_len;
    size_t body_len = wr - header_len;
    /* If this is the first segment, skip the initial RangingHeader inside the body. */
    size_t offset = 0;
    if (proc->segmentation_header_.first_segment && body_len > RAS_RANGING_HEADER_SIZE)
	    offset = RAS_RANGING_HEADER_SIZE;
    if (body_len >= offset + RAS_SUBEVENT_HEADER_SIZE) {
	    dbg_decode_ras_subevent_header(rap, body + offset, RAS_SUBEVENT_HEADER_SIZE);
    }
#endif
    bool ok = bt_gatt_server_send_notification(server, handle, seg, wr, false);
    free(seg);
    if (!ok) {
      DBG(rap,"Failed to send RAS notification");
      return;
    }
    /* Advance read cursor and update segmentation state */
    proc->ras_raw_data_index_ += copy_size;
    proc->segmentation_header_.first_segment = 0;
    proc->segmentation_header_.rolling_segment_counter =
        (uint8_t)((proc->segmentation_header_.rolling_segment_counter + 1) & 0x3F);
    if (proc->segmentation_header_.last_segment || proc->ras_raw_data_index_ >= proc->ras_raw_data_.len) {
      DBG(rap,"RAS clear ras buffers ");
      proc->ras_raw_data_.len = 0;
      proc->ras_raw_data_index_ = 0;
      proc->ranging_header_prepended_ = false;
      return;
    }
    // Continue loop for next segment
  }
}
static inline void resptracker_reset_current_proc(struct cstracker *t) {
    if (!t) return;
    if (t->current_proc) {
        cs_procedure_data_destroy(t->current_proc);
        t->current_proc = NULL;
    }
}

/* Unified local subevent handler */
static void handle_local_subevent_result(struct bt_rap *rap,
                                         bool has_header_fields,
                                         uint8_t config_id,
                                         uint8_t num_ant_paths,
                                         uint16_t proc_counter,
                                         uint16_t start_acl_conn_evt_counter,
                                         uint16_t freq_comp,
                                         int8_t  ref_pwr_lvl,
                                         uint8_t proc_done_status,
                                         uint8_t subevt_done_status,
                                         uint8_t abort_reason,
                                         uint8_t num_steps_reported,
                                         const void *step_bytes,
                                         uint16_t step_bytes_len)
{
    if (!rap || !rap->resptracker || !step_bytes)
        return;
    struct cstracker *resptracker = rap->resptracker;
    bool handle_role = false;
    if (resptracker->current_proc) {
	    CsProcedureData *cur = resptracker->current_proc;
	    if (has_header_fields && cur->counter != proc_counter) {
		    /* Safety: a new procedure; destroy the previous one */
		    resptracker_reset_current_proc(resptracker);
	    }
    }
    CsProcedureData *proc = resptracker->current_proc;
    /* Cache any header info from a RESULT event for later CONT usage */
    if (has_header_fields) {
        g_last_proc_counter = proc_counter;
        g_last_start_acl_conn_evt_counter = start_acl_conn_evt_counter;
        g_last_freq_comp = freq_comp;
        g_last_ref_pwr_lvl = ref_pwr_lvl;
    }
    /* Create the procedure on first use */
    if (!proc) {
        uint16_t create_counter = has_header_fields ? proc_counter : g_last_proc_counter;
        proc = cs_procedure_data_create(create_counter,
                                        num_ant_paths,
                                        config_id,
                                        resptracker->selected_tx_power);
        if (!proc)
            return;
        resptracker->current_proc = proc;
        /* Reference power levels and status defaults */
        cs_pd_set_reference_power_levels(proc,
                                         has_header_fields ? ref_pwr_lvl : g_last_ref_pwr_lvl,
                                         has_header_fields ? ref_pwr_lvl : g_last_ref_pwr_lvl);
        cs_pd_set_local_status(proc, (CsProcedureDoneStatus)proc_done_status);
        cs_pd_set_remote_status(proc, (CsProcedureDoneStatus)subevt_done_status);
    }
    if(resptracker->role == CS_ROLE_REFLECTOR)
     handle_role = false;
    else handle_role = true;
    /* Begin a new RAS subevent only when we have header fields (RESULT event) */
    if (has_header_fields) {
        cs_pd_ras_begin_subevent(proc,
                                 start_acl_conn_evt_counter,
                                 freq_comp,
                                 ref_pwr_lvl);
    }
    /* step_bytes points to an array of struct cs_step_data, not wire format */
    const struct cs_step_data *steps = (const struct cs_step_data *)step_bytes;
    
    /* Process each step */
    for (uint8_t idx = 0; idx < num_steps_reported; idx++) {
        const struct cs_step_data *step = &steps[idx];
        
        const uint8_t mode = step->step_mode;
        const uint8_t channel = step->step_chnl;
        const uint8_t payload_len = step->step_data_length;
        
        DBG(rap,"step[%u]: mode=%u channel=%u payload_len=%u", 
            idx, mode, channel, payload_len);
        
        /* Slim-step serialization for RAS:
         * - 1 byte: mode (bit7 set if aborted)
         * - payload: exactly step_data_length bytes (raw mode data)
         */
        uint8_t mode_byte = step->step_mode;
        const uint8_t *payload = (const uint8_t *)&step->step_mode_data;
        uint8_t plen = step->step_data_length;
        
        if (plen == 0) {
            mode_byte |= RAS_STEP_ABORTED_BIT;  /* set step aborted */
            cs_pd_ras_append_subevent_bytes(proc, &mode_byte, 1);
            /* No payload when aborted */
            DBG(rap,"step[%u]: mode=%u aborted", idx, (unsigned)(mode_byte & 0x7F));
        } else {
            /* Mode byte first */
            cs_pd_ras_append_subevent_bytes(proc, &mode_byte, 1);
            /* Then the raw payload bytes */
            cs_pd_ras_append_subevent_bytes(proc, payload, plen);
            DBG(rap,"step[%u]: mode=%u payload_len=%u", idx, (unsigned)mode, (unsigned)plen);
        }
    }
    /* Update status for this chunk */
    cs_pd_set_local_status(proc, (CsProcedureDoneStatus)proc_done_status);
    cs_pd_set_remote_status(proc, (CsProcedureDoneStatus)subevt_done_status);
    /* Commit subevent chunk (RESULT or CONT) */
    cs_pd_ras_commit_subevent(proc,
        num_steps_reported,
        proc_done_status,
        subevt_done_status,
        abort_reason & 0x0F,
        (abort_reason >> 4) & 0x0F);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
dbg_hexdump_line(rap, "RAS raw data after commit",
                 (const uint8_t*)proc->ras_raw_data_.data,
                 proc->ras_raw_data_.len,
                 64);
/* Debug: decode the most recent subevent header if we have enough bytes */
    if (proc->ras_raw_data_.len >= RAS_SUBEVENT_HEADER_SIZE) {
        size_t hdr_offset = proc->ras_raw_data_.len - proc->ras_subevent_data_.len
                            - RAS_SUBEVENT_HEADER_SIZE;
        /* Defensive clamp */
        if (hdr_offset + RAS_SUBEVENT_HEADER_SIZE <= proc->ras_raw_data_.len) {
            const uint8_t *hbuf =
                (const uint8_t*)proc->ras_raw_data_.data + hdr_offset;
            dbg_decode_ras_subevent_header(rap, hbuf, RAS_SUBEVENT_HEADER_SIZE);
        } else {
            DBG(rap,"RAS SubeventHeader: could not locate header in raw buffer");
        }
    }
#endif
    /* Ensure first segment body starts with the 4-byte RangingHeader */
    ras_maybe_prepend_ranging_header(proc);
  if (subevt_done_status != SUBEVENT_DONE_PARTIAL_RESULTS)
    /* Send RAS raw segment data */
    send_ras_segment_data(rap, proc);
    /* Procedure complete? Clean up */ 
    if (proc_done_status == CS_PROC_ALL_RESULTS_COMPLETE) {
	DBG(rap,"Destroying CsProcedureData counter=%u and clearing current_proc", proc->counter);
	resptracker_reset_current_proc(resptracker);
	g_last_proc_counter = 0;
	g_last_start_acl_conn_evt_counter = 0;
	g_last_freq_comp = 0;
	g_last_ref_pwr_lvl = 0;
    }
}
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
/* ---- Debug decode helpers for on‑wire headers ---- */
static const char *ranging_done_str(uint8_t s)
{
	switch (s & 0x0F) {
	case RANGING_DONE_ALL_RESULTS_COMPLETE:
		return "ALL_RESULTS_COMPLETE";
	case RANGING_DONE_PARTIAL_RESULTS:
		return "PARTIAL_RESULTS";
	case RANGING_DONE_ABORTED:
		return "ABORTED";
	default:
		return "UNKNOWN";
	}
}

static const char *subevent_done_str(uint8_t s)
{
	switch (s & 0x0F) {
	case SUBEVENT_DONE_ALL_RESULTS_COMPLETE:
		return "ALL_RESULTS_COMPLETE";
	case SUBEVENT_DONE_ABORTED:
		return "ABORTED";
	default:
		return "UNKNOWN";
	}
}

static const char *ranging_abort_str(uint8_t r)
{
	switch (r & 0x0F) {
	case RANGING_ABORT_NO_ABORT:
		return "NO_ABORT";
	case RANGING_ABORT_LOCAL_HOST_OR_REMOTE:
		return "LOCAL_OR_REMOTE";
	case RANGING_ABORT_INSUFFICIENT_FILTERED_CHANNELS:
		return "INSUFF_CHANNELS";
	case RANGING_ABORT_INSTANT_HAS_PASSED:
		return "INSTANT_PASSED";
	case RANGING_ABORT_UNSPECIFIED:
		return "UNSPECIFIED";
	default:
		return "UNKNOWN";
	}
}

static const char *subevent_abort_str(uint8_t r)
{
	switch (r & 0x0F) {
	case SUBEVENT_ABORT_NO_ABORT:
		return "NO_ABORT";
	case SUBEVENT_ABORT_LOCAL_HOST_OR_REMOTE:
		return "LOCAL_OR_REMOTE";
	case SUBEVENT_ABORT_NO_CS_SYNC_RECEIVED:
		return "NO_CS_SYNC";
	case SUBEVENT_ABORT_SCHEDULING_CONFLICTS_OR_LIMITED_RESOURCES:
		return "SCHED_CONFLICT_OR_RES";
	case SUBEVENT_ABORT_UNSPECIFIED:
		return "UNSPECIFIED";
	default:
		return "UNKNOWN";
	}
}

/* Decode and print a RAS subevent header from on-wire 8-byte format */
static void dbg_decode_ras_subevent_header(struct bt_rap *rap,
					const uint8_t *buf, size_t len)
{
	uint16_t start_acl_conn_event;
	uint16_t freq_comp;
	uint8_t done_byte;
	uint8_t abort_byte;
	int8_t ref_pwr_lvl;
	uint8_t num_steps;
	uint8_t ranging_done_status;
	uint8_t subevent_done_status;
	uint8_t ranging_abort_reason;
	uint8_t subevent_abort_reason;

	if (!buf || len < RAS_SUBEVENT_HEADER_SIZE) {
		DBG(rap, "RAS SubeventHeader: <invalid len=%zu>", len);
		return;
	}

	start_acl_conn_event = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
	freq_comp = (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8));
	done_byte = buf[4];
	abort_byte = buf[5];
	ref_pwr_lvl = (int8_t)buf[6];
	num_steps = buf[7];
	ranging_done_status = (uint8_t)(done_byte & 0x0F);
	subevent_done_status = (uint8_t)((done_byte >> 4) & 0x0F);
	ranging_abort_reason = (uint8_t)(abort_byte & 0x0F);
	subevent_abort_reason = (uint8_t)((abort_byte >> 4) & 0x0F);

	DBG(rap, "RAS SubeventHeader: start_acl_evt=%u freq_comp=%d "
		"ranging_done=%u(%s) subevt_done=%u(%s) "
		"ranging_abort=%u(%s) subevt_abort=%u(%s) "
		"ref_pwr_lvl=%d num_steps=%u",
		start_acl_conn_event,
		(int16_t)freq_comp,
		ranging_done_status, ranging_done_str(ranging_done_status),
		subevent_done_status, subevent_done_str(subevent_done_status),
		ranging_abort_reason, ranging_abort_str(ranging_abort_reason),
		subevent_abort_reason,
		subevent_abort_str(subevent_abort_reason),
		ref_pwr_lvl,
		num_steps);
}

static void dbg_decode_ranging_header(struct bt_rap *rap,
					const uint8_t *buf, size_t len)
{
	uint16_t rcid;
	uint16_t ranging_counter;
	uint8_t configuration_id;
	int8_t selected_tx_power;
	uint8_t b3;
	uint8_t antenna_paths_mask;
	uint8_t pct_format;
	const char *pct_str;

	if (!buf || len < RAS_RANGING_HEADER_SIZE) {
		DBG(rap, "RangingHeader: <invalid len=%zu>", len);
		return;
	}

	/* Little-endian 16 bits: [ranging_counter(12) | config_id(4)] */
	rcid = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
	ranging_counter = (uint16_t)(rcid & 0x0FFF);
	configuration_id = (uint8_t)((rcid >> 12) & 0x0F);
	selected_tx_power = (int8_t)buf[2];
	b3 = buf[3];
	antenna_paths_mask = (uint8_t)(b3 & 0x0F);
	pct_format = (uint8_t)((b3 >> 6) & 0x03);
	pct_str = (pct_format == IQ) ? "IQ" :
		(pct_format == PHASE) ? "PHASE" : "UNKNOWN";

	DBG(rap, "RangingHeader: rc=%u cfg_id=%u tx_pwr=%d dBm "
		"ant_mask=0x%01x pct_format=%u(%s)",
		ranging_counter,
		configuration_id,
		selected_tx_power,
		antenna_paths_mask,
		pct_format, pct_str);
}

/* Compact hexdump helper: prints up to max bytes on one DBG line */
static void dbg_hexdump_line(struct bt_rap *rap, const char *prefix,
				const uint8_t *buf, size_t len, size_t max)
{
	size_t n;
	char line[3 * 1024]; /* supports up to max=128 safely */
	size_t wr = 0;
	size_t i;

	if (!buf || len == 0) {
		DBG(rap, "%s: <empty>", prefix ? prefix : "hexdump");
		return;
	}

	n = len < max ? len : max;

	for (i = 0; i < n && (wr + 3) < sizeof(line); i++)
		wr += snprintf(line + wr, sizeof(line) - wr, "%02x ", buf[i]);

	if (wr > 0)
		line[wr - 1] = '\0'; /* trim trailing space */

	DBG(rap, "%s (%zu/%zu): %s%s",
		prefix ? prefix : "hexdump",
		n, len, line, (len > n ? " ..." : ""));
}

static inline const char *proc_status_str(uint8_t s)
{
    /* Adjust mapping to actual enum values if available */
    switch (s) {
    case CS_PROC_PARTIAL_RESULTS: return "PARTIAL";
    case CS_PROC_ALL_RESULTS_COMPLETE: return "COMPLETE";
    default: return "UNKNOWN";
    }
}

static void dbg_print_step(struct bt_rap *rap, const struct cs_step_data *s, uint8_t idx)
{
    if (!s) return;
    /* Extract mode_type (bits 0-1) and aborted flag (bit 7) */
    uint8_t mode_type = s->step_mode & 0x03;
    uint8_t aborted = (s->step_mode & 0x80) ? 1 : 0;
    DBG(rap,"  step[%x]: mode=%x (type=%u%s) chnl=%x len=%x",
        idx, s->step_mode, mode_type, aborted ? " ABORTED" : "", 
        s->step_chnl, s->step_data_length);
    /* Dump the raw mode payload bounded by the reported length */
    const uint8_t *payload = (const uint8_t *)&s->step_mode_data;
    size_t payload_len = s->step_data_length;
    if (payload_len > sizeof(union cs_mode_data))
        payload_len = sizeof(union cs_mode_data);
    dbg_hexdump_line(rap, "    mode_data", payload, payload_len, 64);
    /* Decode per-mode fields for readability (best-effort) */
    switch (mode_type) {
    case 0: {
        const struct cs_mode_zero_data *z = &s->step_mode_data.mode_zero_data;
        DBG(rap,"    m0: ant=%x pQual=%x rssi_dbm=%d",
            z->packet_ant, z->packet_quality, (int8_t)z->packet_rssi_dbm);
        break;
    }
    case 1: {
        const struct cs_mode_one_data *o = &s->step_mode_data.mode_one_data;
        DBG(rap,"    m1: ant=%x pQual=%x rssi_dbm=%d nadm=%x",
            o->packet_ant, o->packet_quality, (int8_t)o->packet_rssi_dbm, o->packet_nadm);
        DBG(rap,"        toaTodInit=%x todToaRefl=%x pct1=(%x,%x) pct2=(%x,%x)",
            o->toa_tod_init, o->tod_toa_refl,
            o->packet_pct1.i_sample, o->packet_pct1.q_sample,
            o->packet_pct2.i_sample, o->packet_pct2.q_sample);
        break;
    }
    case 2: {
        const struct cs_mode_two_data *t = &s->step_mode_data.mode_two_data;
        DBG(rap,"    m2: antPermIdx=%x", t->ant_perm_index);
        /* If available in your struct, dump pct_format/RPL/tone quality here */
        break;
    }
    case 3: {
        const struct cs_mode_three_data *th = &s->step_mode_data.mode_three_data;
        const struct cs_mode_one_data *o = &th->mode_one_data;
        const struct cs_mode_two_data *t = &th->mode_two_data;
        DBG(rap,"    m3: (m1 subset) ant=%x pQual=%x rssi_dbm=%d nadm=%x "
            "toaTodInit=%x todToaRefl=%x pct1=(%x,%x) pct2=(%x,%x); (m2 subset) antPermIdx=%x",
            o->packet_ant, o->packet_quality, (int8_t)o->packet_rssi_dbm, o->packet_nadm,
            o->toa_tod_init, o->tod_toa_refl,
            o->packet_pct1.i_sample, o->packet_pct1.q_sample,
            o->packet_pct2.i_sample, o->packet_pct2.q_sample,
            t->ant_perm_index);
        break;
    }
    default:
        DBG(rap,"    mode_type=%u decode not implemented (raw_mode=0x%02x)", 
            mode_type, s->step_mode);
        break;
    }
}
static void dbg_rap_ev_cs_subevent_result(struct bt_rap *rap, const struct rap_ev_cs_subevent_result *r,
                                           uint16_t length)
{
    if (!r) return;
    size_t base_len = offsetof(struct rap_ev_cs_subevent_result, step_data);
    if (length < base_len) {
        DBG(rap,"CS RESULT: invalid len=%u (<%zu)", length, base_len);
        return;
    }
    uint8_t proc_abort_reason   = r->abort_reason & 0x0F;
    uint8_t subevt_abort_reason = (r->abort_reason >> 4) & 0x0F;
    DBG(rap,"CS RESULT: conn_hdl=0x%04x cfg_id=%x start_acl_conn_evt=%x "
        "proc_counter=%x freq_comp=%x ref_pwr_lvl=%x "
        "proc_status=%x(%s) subevt_status=%x abort=0x%02x "
        "abort(proc=%x, subevt=%x) ant_paths=%x steps=%x len=%x",
        r->conn_hdl, r->config_id, r->start_acl_conn_evt_counter,
        r->proc_counter, r->freq_comp, r->ref_pwr_lvl,
        r->proc_done_status, proc_status_str(r->proc_done_status),
        r->subevt_done_status, r->abort_reason,
        proc_abort_reason, subevt_abort_reason,
        r->num_ant_paths, r->num_steps_reported, length);
    const struct cs_step_data *step_array = r->step_data;
    for (uint8_t i = 0; i < r->num_steps_reported; i++) {
        /* Access each step directly from the array - they're fixed-size structures */
        const struct cs_step_data *step = &step_array[i];
        dbg_print_step(rap, step, i);
    }
    /* Compact raw event dump (truncate to keep logs readable) */
    dbg_hexdump_line(rap, "  event_raw", (const uint8_t *)r, length, 128);
}
static void dbg_rap_ev_cs_subevent_result_cont(struct bt_rap *rap, const struct rap_ev_cs_subevent_result_cont *c,
                                                uint16_t length)
{
    if (!c) return;
    size_t base_len = offsetof(struct rap_ev_cs_subevent_result_cont, step_data);
    if (length < base_len) {
        DBG(rap,"CS RESULT CONT: invalid len=%u (<%zu)", length, base_len);
        return;
    }
    uint8_t proc_abort_reason   = c->abort_reason & 0x0F;
    uint8_t subevt_abort_reason = (c->abort_reason >> 4) & 0x0F;
    DBG(rap,"CS RESULT CONT: cfg_id=%u ant_paths=%u "
        "proc_status=%u(%s) subevt_status=%u abort=0x%02x "
        "abort(proc=%u, subevt=%u) steps=%u len=%u",
        c->config_id, c->num_ant_paths,
        c->proc_done_status, proc_status_str(c->proc_done_status),
        c->subevt_done_status, c->abort_reason,
        proc_abort_reason, subevt_abort_reason,
        c->num_steps_reported, length);
    const struct cs_step_data *step_array = c->step_data;
    for (uint8_t i = 0; i < c->num_steps_reported; i++) {
        /* Access each step directly from the array - they're fixed-size structures */
        const struct cs_step_data *step = &step_array[i];
        dbg_print_step(rap, step, i);
    }
    dbg_hexdump_line(rap, "  event_raw_cont", (const uint8_t *)c, length, 128);
}
#endif
static void form_ras_data_with_cs_subevent_result(struct bt_rap *rap,
        const struct rap_ev_cs_subevent_result *data,
        uint16_t length)
{
    if (!rap || !rap->resptracker || !data)
        return;
    /* Defensive check: base header must be present */
    size_t base_len = offsetof(struct rap_ev_cs_subevent_result, step_data);
    if (length < base_len)
        return;
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
    dbg_rap_ev_cs_subevent_result(rap, data, length);
#endif
    DBG(rap,"Received CS subevent result subevent: len=%d", length);
    uint16_t step_bytes_len = (uint16_t)(length - base_len);
    handle_local_subevent_result(rap,
        true,                          /* has header fields */
        data->config_id,
        data->num_ant_paths,
        data->proc_counter,
        data->start_acl_conn_evt_counter,
        data->freq_comp,
        data->ref_pwr_lvl,
        data->proc_done_status,
        data->subevt_done_status,
        data->abort_reason,
        data->num_steps_reported,
	data->step_data,            /* start of steps */
        step_bytes_len);            /* total bytes available for steps */
}
static void form_ras_data_with_cs_subevent_result_cont(struct bt_rap *rap,
        const struct rap_ev_cs_subevent_result_cont *cont,
        uint16_t length)
{
    if (!rap || !rap->resptracker || !cont)
        return;
    size_t base_len = offsetof(struct rap_ev_cs_subevent_result_cont, step_data);
    if (length < base_len)
        return;
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
    dbg_rap_ev_cs_subevent_result_cont(rap, cont, length);
#endif
    DBG(rap,"Received CS subevent result continue subevent: len=%d", length);
    uint16_t step_bytes_len = (uint16_t)(length - base_len);
    /* Use cached header values captured from the last RESULT event */
    handle_local_subevent_result(rap,
        false,                         /* CONT has no header fields */
        cont->config_id,
        cont->num_ant_paths,
        g_last_proc_counter,
        g_last_start_acl_conn_evt_counter,
        g_last_freq_comp,
        g_last_ref_pwr_lvl,
        cont->proc_done_status,
        cont->subevt_done_status,
        cont->abort_reason,
        cont->num_steps_reported,
        cont->step_data,
        step_bytes_len);            /* total bytes available for steps */
}

void bt_rap_hci_cs_subevent_result_cont_callback(uint16_t length,
						const void *param,
						void *user_data)
{
    const struct rap_ev_cs_subevent_result_cont *cont = param;
	struct bt_rap *rap = user_data;

	DBG(rap, "Received CS subevent CONT: len=%d", length);
    /* Basic defensive length check (base struct without step_data) */
    if (length < offsetof(struct rap_ev_cs_subevent_result_cont, step_data)) {
        DBG(rap,"CONT event too short: %u", length);
        return;
    }
    form_ras_data_with_cs_subevent_result_cont(rap, cont, length);
}

void bt_rap_hci_cs_subevent_result_callback(uint16_t length,
					const void *param,
					void *user_data)
{
const struct rap_ev_cs_subevent_result *data = param;
   struct bt_rap *rap = user_data;
   DBG(rap,"Received CS subevent: len=%d", length);
   /* Populate CsProcedureData and send RAS payload */
   form_ras_data_with_cs_subevent_result(rap, data, length);
}

void bt_rap_hci_cs_procedure_enable_complete_callback(uint16_t length,
						const void *param,
						void *user_data)
{
 const struct rap_ev_cs_proc_enable_cmplt *data = param;
   struct bt_rap *rap = user_data;
   DBG(rap,"Received CS procedure enable complete subevent: len=%d", length);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
   DBG(rap,"status=0x%02x conn_hdl=0x%04x config_id=%d state=%d tone_ant_config_sel=%d sel_tx_pwr=%d sub_evt_len=%d",
    data->status,
    data->conn_hdl,
    data->config_id,
    data->state,
    data->tone_ant_config_sel,
    data->sel_tx_pwr,
    data->sub_evt_len);
   DBG(rap,"sub_evts_per_evt=%d sub_evt_intrvl=0x%04x evt_intrvl=0x%04x proc_intrvl=0x%04x proc_counter=%d max_proc_len=%d",
    data->sub_evts_per_evt,
    data->sub_evt_intrvl,
    data->evt_intrvl,
    data->proc_intrvl,
    data->proc_counter,
    data->max_proc_len);
#endif
   if (!rap->resptracker) {
        struct cstracker *resptracker = new0(struct cstracker, 1);
        cs_tracker_init(resptracker, data->conn_hdl);
        rap->resptracker = resptracker;
   }
   struct cstracker *resptracker = rap->resptracker;
   /* Populate responder tracker */
   resptracker->config_id = data->config_id;
   resptracker->selected_tx_power = data->sel_tx_pwr;
}

void bt_rap_hci_cs_sec_enable_complete_callback(uint16_t length,
						 const void *param,
						 void *user_data)
{
const struct rap_ev_cs_sec_enable_cmplt *data = param;
   struct bt_rap *rap = user_data;
   DBG(rap,"Received CS security enable subevent: len=%d", length);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
   DBG(rap,"status=0x%x", data->status);
   DBG(rap,"conn_hdl=0x%x",data->conn_hdl);
#endif

}

void bt_rap_hci_cs_config_complete_callback(uint16_t length,
					const void *param,
					void *user_data)
{
 const struct rap_ev_cs_config_cmplt *data = param;
   struct bt_rap *rap = user_data;
   if (!rap)
        return;
   DBG(rap,"Received CS config complete subevent: len=%d", length);
#ifdef EXTRA_LOGGING_DUMPING_RAW_BYTES
   DBG(rap,"status=0x%02x", data->status);
   DBG(rap,"conn_hdl=0x%04x",data->conn_hdl);
   DBG(rap,"config_id=0x%d", data->config_id);
   DBG(rap,"action=0x%x", data->action);
   DBG(rap,"main_mode_type=0x%x", data->main_mode_type);
   DBG(rap,"sub_mode_type=0x%x", data->sub_mode_type);
   DBG(rap,"min_main_mode_steps=0x%x", data->min_main_mode_steps);
   DBG(rap,"max_main_mode_steps=0x%x", data->max_main_mode_steps);
   DBG(rap,"main_mode_rep=0x%x", data->main_mode_rep);
   DBG(rap,"mode_0_steps=0x%x", data->mode_0_steps);
   DBG(rap,"role=0x%x", data->role);
   DBG(rap,"rtt_type=0x%x", data->rtt_type);
   DBG(rap,"cs_sync_phy=0x%x", data->cs_sync_phy);
   for (int i = 0; i < 10; i++) {
       DBG(rap,"channel_map[%d]=0x%x", i, data->channel_map[i]);
   }
   DBG(rap,"channel_map_rep=0x%x", data->channel_map_rep);
   DBG(rap,"channel_sel_type=0x%x", data->channel_sel_type);
   DBG(rap,"ch3c_shape=0x%x", data->ch3c_shape);
   DBG(rap,"ch3c_jump=0x%x", data->ch3c_jump);
   DBG(rap,"reserved=0x%x", data->reserved);
   DBG(rap,"t_ip1_time=0x%x", data->t_ip1_time);
   DBG(rap,"t_ip2_time=0x%x", data->t_ip2_time);
   DBG(rap,"t_fcs_time=0x%x", data->t_fcs_time);
   DBG(rap,"t_pm_time=0x%x", data->t_pm_time);
#endif
   if (!rap->resptracker) {
        struct cstracker *resptracker = new0(struct cstracker, 1);
        cs_tracker_init(resptracker, data->conn_hdl);
        rap->resptracker = resptracker;
   }
   struct cstracker *resptracker = rap->resptracker;
   /* Basic fields */
   resptracker->config_id = data->config_id;
   resptracker->role = data->role;
}

struct bt_rap *bt_rap_new(struct gatt_db *ldb, struct gatt_db *rdb)
{
	struct bt_rap *rap;
	struct bt_rap_db *rapdb;

	if (!ldb)
		return NULL;

	rapdb = rap_get_db(ldb);
	if (!rapdb)
		return NULL;

	rap = new0(struct bt_rap, 1);
	rap->lrapdb = rapdb;
	rap->pending = queue_new();
	rap->ready_cbs = queue_new();
	rap->notify = queue_new();

	if (!rdb)
		goto done;

	rapdb = new0(struct bt_rap_db, 1);
	rapdb->db = gatt_db_ref(rdb);

	rap->rrapdb = rapdb;

done:
	bt_rap_ref(rap);

	return rap;
}

static void foreach_rap_char(struct gatt_db_attribute *attr, void *user_data)
{
	struct bt_rap *rap = user_data;
	uint16_t value_handle;
	bt_uuid_t uuid;
	bt_uuid_t uuid_features;
	bt_uuid_t uuid_realtime;
	bt_uuid_t uuid_ondemand;
	bt_uuid_t uuid_cp;
	bt_uuid_t uuid_dataready;
	bt_uuid_t uuid_overwritten;
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
		DBG(rap, "Features characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->feat_chrc)
			return;

		ras->feat_chrc = attr;
	}

	if (!bt_uuid_cmp(&uuid, &uuid_realtime)) {
		DBG(rap, "Real Time Data characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->realtime_chrc)
			return;

		ras->realtime_chrc = attr;
	}

	if (!bt_uuid_cmp(&uuid, &uuid_ondemand)) {
		DBG(rap, "On-demand Data characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->ondemand_chrc)
			return;

		ras->ondemand_chrc = attr;
	}

	if (!bt_uuid_cmp(&uuid, &uuid_cp)) {
		DBG(rap, "Control Point characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->cp_chrc)
			return;

		ras->cp_chrc = attr;
	}

	if (!bt_uuid_cmp(&uuid, &uuid_dataready)) {
		DBG(rap, "Data Ready characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->ready_chrc)
			return;

		ras->ready_chrc = attr;
	}

	if (!bt_uuid_cmp(&uuid, &uuid_overwritten)) {
		DBG(rap, "Overwritten characteristic found: handle 0x%04x",
		    value_handle);

		ras = rap_get_ras(rap);
		if (!ras || ras->overwritten_chrc)
			return;

		ras->overwritten_chrc = attr;
	}
}

static void foreach_rap_service(struct gatt_db_attribute *attr,
				void *user_data)
{
	struct bt_rap *rap = user_data;
	struct ras *ras = rap_get_ras(rap);

	ras->svc = attr;

	gatt_db_service_set_claimed(attr, true);

	gatt_db_service_foreach_char(attr, foreach_rap_char, rap);
}

unsigned int bt_rap_ready_register(struct bt_rap *rap,
				   bt_rap_ready_func_t func, void *user_data,
				   bt_rap_destroy_func_t destroy)
{
	struct bt_rap_ready *ready;
	static unsigned int id;

	if (!rap)
		return 0;

	DBG(rap, "bt_rap_ready_register");

	ready = new0(struct bt_rap_ready, 1);
	ready->id = ++id ? id : ++id;
	ready->func = func;
	ready->destroy = destroy;
	ready->data = user_data;

	queue_push_tail(rap->ready_cbs, ready);

	return ready->id;
}

static bool match_ready_id(const void *data, const void *match_data)
{
	const struct bt_rap_ready *ready = data;
	unsigned int id = PTR_TO_UINT(match_data);

	return ready->id == id;
}

bool bt_rap_ready_unregister(struct bt_rap *rap, unsigned int id)
{
	struct bt_rap_ready *ready;

	ready = queue_remove_if(rap->ready_cbs, match_ready_id,
				UINT_TO_PTR(id));
	if (!ready)
		return false;

	rap_ready_free(ready);

	return true;
}

static struct bt_rap *bt_rap_ref_safe(struct bt_rap *rap)
{
	if (!rap || !rap->ref_count)
		return NULL;

	return bt_rap_ref(rap);
}

static void rap_notify_ready(struct bt_rap *rap)
{
	const struct queue_entry *entry;

	if (!bt_rap_ref_safe(rap))
		return;

	for (entry = queue_get_entries(rap->ready_cbs); entry;
	     entry = entry->next) {
		struct bt_rap_ready *ready = entry->data;

		ready->func(rap, ready->data);
	}

	bt_rap_unref(rap);
}

static void rap_idle(void *data)
{
	struct bt_rap *rap = data;

	rap->idle_id = 0;
	rap_notify_ready(rap);
}

bool bt_rap_attach(struct bt_rap *rap, struct bt_gatt_client *client)
{
	bt_uuid_t uuid;

	if (!sessions)
		sessions = queue_new();

	queue_push_tail(sessions, rap);

	if (!client)
		return true;

	if (rap->client)
		return false;

	rap->client = bt_gatt_client_clone(client);
	if (!rap->client)
		return false;

	bt_gatt_client_idle_register(rap->client, rap_idle, rap, NULL);

	bt_uuid16_create(&uuid, RAS_UUID16);

	gatt_db_foreach_service(rap->lrapdb->db, &uuid,
				foreach_rap_service, rap);

	return true;
}
