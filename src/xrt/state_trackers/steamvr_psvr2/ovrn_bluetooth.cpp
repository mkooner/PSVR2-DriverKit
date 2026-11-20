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

	// BlueZ names a device /org/bluez/hciN/dev_XX_XX_XX_XX_XX_XX; try the adapters in turn.
	std::string dev = "dev_" + address;
	for (char &c : dev) {
		c = c == ':' ? '_' : (char)std::toupper((unsigned char)c);
	}
	bool done = false;
	std::string last = "no adapter has it";
	for (int adapter = 0; adapter < 4 && !done; adapter++) {
		char path[96];
		snprintf(path, sizeof(path), "/org/bluez/hci%d/%s", adapter, dev.c_str());
		DBusMessage *msg = dbus_message_new_method_call("org.bluez", path, "org.bluez.Device1", "Disconnect");
		if (msg == nullptr) {
			last = "out of memory";
			break;
		}
		DBusMessage *reply = dbus_connection_send_with_reply_and_block(conn, msg, 3000, &err);
		dbus_message_unref(msg);
		if (reply != nullptr) {
			dbus_message_unref(reply);
			done = true;
		} else {
			last = err.message != nullptr ? err.message : "?";
			dbus_error_free(&err);
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
