cmd_/home/rnd/Desktop/20201202_LINUX_BT_DRIVER/usb/bluetooth_usb_driver/Module.symvers := sed 's/\.ko$$/\.o/' /home/rnd/Desktop/20201202_LINUX_BT_DRIVER/usb/bluetooth_usb_driver/modules.order | scripts/mod/modpost -m -a  -o /home/rnd/Desktop/20201202_LINUX_BT_DRIVER/usb/bluetooth_usb_driver/Module.symvers -e -i Module.symvers   -T -