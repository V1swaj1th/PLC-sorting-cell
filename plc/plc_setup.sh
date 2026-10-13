#!/usr/bin/env bash
set -euo pipefail
C=openplc-runtime
CFG=/workdir/core/src/drivers/plugins/python/modbus_slave/modbus_slave_config.json

docker exec "$C" sed -i '/^modbus_slave,/ s/,0,0,/,1,0,/' /workdir/plugins.conf
docker exec -i "$C" tee "$CFG" > /dev/null << 'JSON'
{
  "network_configuration": { "host": "0.0.0.0", "port": 5020 },
  "buffer_mapping": {
    "holding_registers": { "qw_count": 1024, "mw_count": 1024, "md_count": 1024, "ml_count": 1024 },
    "coils": { "qx_bits": 8192, "mx_bits": 0 },
    "discrete_inputs": { "ix_bits": 8192 },
    "input_registers": { "iw_count": 1024 }
  }
}
JSON
docker restart "$C" > /dev/null
sleep 10
if ss -ltn | grep ':5020' > /dev/null; then
  echo "OK: Modbus slave listening on 5020"
else
  echo "WARNING: nothing on 5020. Check: docker logs openplc-runtime"
fi
