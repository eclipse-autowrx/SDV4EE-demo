# Demonstrator for digital.auto
This is a WIP working documentation:
This means it is neither pretty nor complete but should give quick hints for few topics.

Content is subject to change; please update it when content changes!

## How to build and flash (shortened version)
Use `install_zephyr.sh` for initial install of zephyr environment in WSL.

See file `init.source` for few details how to inisialise your current shell to be able to build/flash a zone.

## IP assignment
To change IP address before build, modify parameteter `CONFIG_NET_CONFIG_MY_IPV4_ADDR` in file `prj.conf`.

### IP assignment of zone ECUs
| Location | IP | Connected Motor |
| --------  | -- | --------------- |
| right | 192.168.88.201 | foc |
| back | 192.168.88.202 | foc |
| front | 192.168.88.203 | IDD |
| left | 192.168.88.204 | unused |

### IP assignment of VIP
* vip: 192.168.88.105
