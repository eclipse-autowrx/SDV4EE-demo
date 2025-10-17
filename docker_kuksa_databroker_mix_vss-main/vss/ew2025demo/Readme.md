# VSS over EW Demo


Just building overlay:

```
vspec export json --vspec ewoverlay.vspec --output ewoverlay.json
```


Building combined model

```
vspec export json --vspec vss5.yaml --overlays ewoverlay.vspec --output vssEWCombined.json
```

# Running in databroker

Assuming starting databroker in this directory

