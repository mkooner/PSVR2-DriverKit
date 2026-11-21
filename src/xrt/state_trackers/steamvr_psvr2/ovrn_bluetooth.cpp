// Copyright 2026, Mehrab Kooner
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Disconnect a Bluetooth device through BlueZ over the system D-Bus, see ovrn_bluetooth.hpp.
 * @ingroup st_ovrn
 */

#include "ovrn_bluetooth.hpp"

#include "xrt/xrt_config_have.h"

#include <cctype>
#include <chrono>
#include <cstdio>

#ifdef XRT_HAVE_DBUS
#include <dbus/dbus.h>
#endif

namespace ovrn {

bool
is_bluetooth_address(const std::string &s)
{
	if (s.size() != 17) {
		return false;
	}
	for (size_t i = 0; i < s.size(); i++) {
		if (i % 3 == 2 ? s[i] != ':' : !std::isxdigit((unsigned char)s[i])) {
			return false;
		}
	}
	return true;
}

bool
bluetooth_disconnect(const std::string &address, std::string *why)
{
	if (!is_bluetooth_address(address)) {
		*why = "not a Bluetooth address";
		return false;
	}
#ifndef XRT_HAVE_DBUS
	*why = "built without D-Bus support";
	return false;
#else
	DBusError err;
	dbus_error_init(&err);
	// A private connection: closed again here, never shared with anything else in vrserver.
	DBusConnection *conn = dbus_bus_get_private(DBUS_BUS_SYSTEM, &err);
	if (conn == nullptr) {
		*why = std::string("no system D-Bus: ") + (err.message != nullptr ? err.message : "?");
		dbus_error_free(&err);
		return false;
	}
	dbus_connection_set_exit_on_disconnect(conn, FALSE);

	/*
	 * BlueZ names a device /org/bluez/hciN/dev_XX_XX_XX_XX_XX_XX. The call goes to every adapter at once and the
	 * answers are awaited for at most 1 s in all: this runs in SteamVR's shutdown, which it aborts after 10 s
	 * (and the next start is then in safe mode without our driver). Trying the adapters in turn with 3 s each could
	 * take 24 s for two controllers when BlueZ was slow to answer.
	 */
	std::string dev = "dev_" + address;
	for (char &c : dev) {
		c = c == ':' ? '_' : (char)std::toupper((unsigned char)c);
	}
	const int adapters = 4;
	DBusPendingCall *pending[adapters] = {};
	for (int adapter = 0; adapter < adapters; adapter++) {
		char path[96];
		snprintf(path, sizeof(path), "/org/bluez/hci%d/%s", adapter, dev.c_str());
		DBusMessage *msg = dbus_message_new_method_call("org.bluez", path, "org.bluez.Device1", "Disconnect");
		if (msg == nullptr) {
			continue;
		}
		if (!dbus_connection_send_with_reply(conn, msg, &pending[adapter], 1000)) {
			pending[adapter] = nullptr;
		}
		dbus_message_unref(msg);
	}
	dbus_connection_flush(conn);

	bool done = false;
	std::string last = "no adapter has it";
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
	for (;;) {
		bool waiting = false;
		for (int adapter = 0; adapter < adapters; adapter++) {
			if (pending[adapter] == nullptr) {
				continue;
			}
			if (!dbus_pending_call_get_completed(pending[adapter])) {
				waiting = true;
				continue;
			}
			DBusMessage *reply = dbus_pending_call_steal_reply(pending[adapter]);
			if (reply != nullptr) {
				if (dbus_message_get_type(reply) == DBUS_MESSAGE_TYPE_METHOD_RETURN) {
					done = true;
				} else if (dbus_set_error_from_message(&err, reply)) {
					last = err.message != nullptr ? err.message : "?";
					dbus_error_free(&err);
				}
				dbus_message_unref(reply);
			}
			dbus_pending_call_unref(pending[adapter]);
			pending[adapter] = nullptr;
		}
		if (done || !waiting || std::chrono::steady_clock::now() >= deadline) {
			break;
		}
		dbus_connection_read_write(conn, 50);
	}
	for (int adapter = 0; adapter < adapters; adapter++) {
		if (pending[adapter] != nullptr) {
			// No answer in time: the call was sent, BlueZ may still carry it out.
			dbus_pending_call_cancel(pending[adapter]);
			dbus_pending_call_unref(pending[adapter]);
			if (!done) {
				last = "BlueZ didn't answer within 1 s (disconnect requested)";
			}
		}
	}
	dbus_connection_close(conn);
	dbus_connection_unref(conn);
	if (!done) {
		*why = last;
	}
	return done;
#endif
}

} // namespace ovrn
