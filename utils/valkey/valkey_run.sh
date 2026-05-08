#!/bin/bash -i

/root/valkey/valkey/src/valkey-server \
  /root/AUSPEX/AUSPEX-KNOW/src/auspex_knowledge/auspex_knowledge/config/valkey_server.conf \
  --bind $AUSPEX_DB_IP 127.0.0.1 -::1 \
  --loadmodule /root/valkey/valkeyJSON/build/src/libjson.so
