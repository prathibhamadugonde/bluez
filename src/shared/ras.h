/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 *
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2025  Qualcomm Incorporated. All rights reserved.
 *
 */
#include <stdbool.h>
#include <inttypes.h>

#include "src/shared/io.h"
#include "src/shared/gatt-client.h"
#include "src/shared/gatt-server.h"


struct bt_ras;

typedef void (*bt_ras_ready_func_t)(struct bt_ras *bt_ras, void *user_data);
typedef void (*bt_ras_destroy_func_t)(void *user_data);
typedef void (*bt_ras_func_t)(struct bt_ras *bt_ras, void *user_data);

struct bt_ras *bt_ras_ref(struct bt_ras *bt_ras);
void bt_ras_unref(struct bt_ras *bt_ras);

void bt_ras_add_db(struct gatt_db *db);

bool bt_ras_attach(struct bt_ras *bt_ras, struct bt_gatt_client *client);
void bt_ras_detach(struct bt_ras *bt_ras);

struct bt_att *bt_ras_get_att(struct bt_ras *bt_ras);

bool bt_ras_set_user_data(struct bt_ras *bt_ras, void *user_data);

/* session related functions */
unsigned int bt_ras_register(bt_ras_func_t attached, bt_ras_func_t detached,
					void *user_data);
unsigned int bt_ras_ready_register(struct bt_ras *bt_ras,
				bt_ras_ready_func_t func, void *user_data,
				bt_ras_destroy_func_t destroy);
bool bt_ras_ready_unregister(struct bt_ras *bt_ras, unsigned int id);

bool bt_ras_unregister(unsigned int id);

struct bt_ras *bt_ras_new(struct gatt_db *ldb, struct gatt_db *rdb);


