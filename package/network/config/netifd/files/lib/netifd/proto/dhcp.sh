#!/bin/sh

. /lib/functions.sh
. ../netifd-proto.sh
init_proto "$@"

proto_dhcp_init_config() {
	proto_config_add_string 'ipaddr:ipaddr'
	proto_config_add_string 'netmask:ipaddr'
	proto_config_add_string 'hostname:hostname'
	proto_config_add_string clientid
	proto_config_add_string vendorid
	proto_config_add_boolean 'broadcast:ipaddr'
	proto_config_add_string 'reqopts:list(string)'
	proto_config_add_string iface6rd
	proto_config_add_string sendopts
	proto_config_add_boolean delegate
}

proto_dhcp_setup() {
	local config="$1"
	local iface="$2"

	local ipaddr hostname clientid vendorid broadcast reqopts iface6rd sendopts delegate
	json_get_vars ipaddr hostname clientid vendorid broadcast reqopts iface6rd sendopts delegate

	local opt dhcpopts
	for opt in $reqopts; do
		append dhcpopts "-O $opt"
	done

	for opt in $sendopts; do
		append dhcpopts "-x $opt"
	done

	[ "$broadcast" = 1 ] && broadcast="-B" || broadcast=
	[ -n "$clientid" ] && clientid="-x 0x3d:${clientid//:/}" || clientid="-C"
	[ -n "$iface6rd" ] && proto_export "IFACE6RD=$iface6rd"
	[ "$delegate" = "0" ] && proto_export "IFACE6RD_DELEGATE=0"

	proto_export "INTERFACE=$config"
	proto_run_command "$config" udhcpc \
		-p /var/run/udhcpc-$iface.pid \
		-s /lib/netifd/dhcp.script \
		-f -t 0 -i "$iface" \
		${ipaddr:+-r $ipaddr} \
		${hostname:+-H $hostname} \
		${vendorid:+-V $vendorid} \
		$clientid $broadcast $dhcpopts
}

proto_dhcp_teardown() {
	local interface="$1"
	proto_kill_command "$interface"
}

add_protocol dhcp

