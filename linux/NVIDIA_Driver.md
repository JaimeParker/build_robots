black nouveau

```shell
sudo gedit /etc/modprobe.d/blacklist.conf
```

twice, first- ctrl + c -  second

add on blacklist.conf

```shell
blacklist nouveau
options nouveau nomodeset=0
```

update core

```shell
sudo update-initramfs -u
```

reboot

```
reboot
```

shutdown desktop service (sudo maybe)

```shell
init 3
```

install driver downloaded from NVIDIA (local), `.run` file

```shell
sudo ./NVIDIAxxx.run --no-opengl-files
```

re-open desktop service

```shell
init 5
```



should be fine, then check `nvidia-smi` to get the recommended `cuda` version then install `cuda` if needed.

