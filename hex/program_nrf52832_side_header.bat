nrfjprog -e
nrfjprog --program ../../../../components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex
nrfjprog --program app_hex/image_transfer_demo_pca10040_s132.hex
nrfjprog -r
