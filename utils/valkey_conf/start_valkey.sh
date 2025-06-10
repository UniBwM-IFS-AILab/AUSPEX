#!/bin/bash -i
# starts the valkey db server. if AUSPEX_DB_IP is empty it is running on local host
/root/valkey/valkey/src/valkey-server $AUSPEX_HOME/utils/valkey_conf/server.conf --bind $AUSPEX_DB_IP 127.0.0.1 -::1 --loadmodule /root/valkey/valkeyJSON/build/src/libjson.so
