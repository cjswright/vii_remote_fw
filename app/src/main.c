/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "hog.h"

LOG_MODULE_REGISTER(vii_remote);


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected %s\n", addr);

	if (bt_conn_set_security(conn, BT_SECURITY_L2)) {
		printk("Failed to set security\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
		       err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed,
#endif
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	hog_init();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};


int16_t acc_xhp, acc_yhp, acc_zhp;
int16_t acc_xf, acc_yf, acc_zf;

/* x = 0.73; a0 = 1 - x, Q.8 */
int a0 = 26;
int b1 = -230;

int16_t rollf, pitchf, yawf;
int16_t rollf_hp, pitchf_hp, yawf_hp;

void
process_accel_data(uint8_t *buf, int len)
{
	if (buf[0] != 0x55)
	{
		printk("Invalid looking SOF: %x\n", buf[0]);
		return;
	}
	if (buf[1] != 0x51)
	{
		printk("Opcode not accel: %x\n", buf[1]);
		return;
	}

	int16_t x = 0, y = 0, z = 0;

	x = buf[2] | (buf[3] << 8);
	y = buf[4] | (buf[5] << 8);
	z = buf[6] | (buf[7] << 8);

	acc_xf = (x * a0 - b1 * acc_xf) >> 8;
	acc_xhp = x - acc_xf;
	acc_yf = (y * a0 - b1 * acc_yf) >> 8;
	acc_yhp = y - acc_yf;
	acc_zf = (z * a0 - b1 * acc_zf) >> 8;
	acc_zhp = z - acc_zf;

	printk("ACCEL x=%d y=%d z=%d\n", acc_xhp, acc_yhp, acc_zhp);
	//hog_update_xy(acc_yhp >> 4, acc_xhp >> 4);
}


void
process_angular_velocity_data(uint8_t *buf, int len)
{
	if (buf[0] != 0x55)
	{
		printk("Invalid looking SOF: %x\n", buf[0]);
		return;
	}
	if (buf[1] != 0x52)
	{
		printk("Opcode not angular velocity: %x\n", buf[1]);
		return;
	}

	int16_t x = 0, y = 0, z = 0;

	x = buf[2] | (buf[3] << 8);
	y = buf[4] | (buf[5] << 8);
	z = buf[6] | (buf[7] << 8);

	//printk("ANGULAR VEL x=%d y=%d z=%d\n", x, y, z);
	//hog_update_xy(x >> 3, y >> 3);
}


void
process_angle_data(uint8_t *buf, int len)
{
	if (buf[0] != 0x55)
	{
		printk("Invalid looking SOF: %x\n", buf[0]);
		return;
	}
	if (buf[1] != 0x53)
	{
		printk("Opcode not angle: %x\n", buf[1]);
		return;
	}

	int16_t roll = 0, pitch = 0, yaw = 0;

	roll = buf[2] | (buf[3] << 8);
	pitch = buf[4] | (buf[5] << 8);
	yaw = buf[6] | (buf[7] << 8);

	rollf = (roll * a0 - b1 * rollf) >> 8;
	rollf_hp = roll - rollf;
	pitchf = (pitch * a0 - b1 * pitchf) >> 8;
	pitchf_hp = pitch - pitchf;
	yawf = (yaw * a0 - b1 * yawf) >> 8;
	yawf_hp = yaw - yawf;

	//printk("ANGLE roll=%d pitch=%d yaw=%d\n", roll, pitch, yaw);
	hog_update_xy(-1 * yawf_hp >> 5, pitchf_hp >> 5);
}


#define NUM_BUFFERS 2
uint8_t buf[NUM_BUFFERS][11];
int buf_index;

RING_BUF_DECLARE(uart_data_buf, 4096);

void
process_uart(void)
{
	uint8_t data[256];
	int read;
	int i = 0;

	while (1)
	{
		read = ring_buf_peek(&uart_data_buf, data, sizeof(data));

		/* Sync for 0x55 */
		while (data[i] != 0x55 && i < read)
			i++;
		ring_buf_get(&uart_data_buf, NULL, i);

		if (i == read)
			return;

		/* Grab the packet */
		read = ring_buf_peek(&uart_data_buf, data, 11);
		if (read != 11)
			return;

		char opc = data[1];
		switch (opc)
		{
			case 0x51:
				process_accel_data(data, 11);
				break;

			case 0x52:
				process_angular_velocity_data(data, 11);
				break;

			case 0x53:
				process_angle_data(data, 11);
				break;

			case 0x54:
				// No idea?
				break;

			default:
				printk("Unknown opcode: %x\n", opc);
		}
		ring_buf_get(&uart_data_buf, NULL, 11);
	}
}


void uart_callback(const struct device *dev,
				   struct uart_event *evt,
				   void *user_data)
{
	switch (evt->type)
	{
		case UART_RX_RDY:
			/* Data in evt->data.rx.buf */
			ring_buf_put(&uart_data_buf, evt->data.rx.buf, evt->data.rx.len);
			process_uart();
			break;

		case UART_RX_BUF_REQUEST:
			//printk("UART_RX_BUF_REQUEST\n");
			buf_index = (buf_index + 1) % NUM_BUFFERS;
			uart_rx_buf_rsp(dev, buf[buf_index], sizeof(buf[buf_index]));
			break;

		case UART_RX_BUF_RELEASED:
			//printk("UART_RX_BUF_RELEASED\n");
			break;

		case UART_RX_STOPPED:
			printk("UART_RX_STOPPED reason=%d\n", evt->data.rx_stop.reason);
			break;

		case UART_RX_DISABLED:
			printk("UART_RX_DISABLED\n");
			break;

		default:
			printk("UART CB %d\n", evt->type);
			break;
	}
}

int main(void)
{
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SAMPLE_BT_USE_AUTHENTICATION)) {
		bt_conn_auth_cb_register(&auth_cb_display);
		printk("Bluetooth authentication callbacks registered.\n");
	}

	const struct device *serial = DEVICE_DT_GET(DT_NODELABEL(arduino_serial));
	err = uart_callback_set(serial, uart_callback, NULL);
	if (err != 0)
	{
		printk("Failed set callback: %d\n", err);
	}

	err = uart_rx_enable(serial, buf[0], sizeof(buf), SYS_FOREVER_US);
	if (err != 0)
	{
		printk("Failed to enable rx: %d\n", err);
	}

	return 0;
}
