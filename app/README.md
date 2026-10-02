# BLE Mesh 物联网控制上位机

## 安装依赖

```powershell
python -m pip install -r requirements.txt
```

## 启动

```powershell
python main.py
```

串口协议与网关固件一致：

- 发送：`AA addr_hi addr_lo type mode 55`
- 节点反馈：文本行 `FB,addr,state`
- 网关状态：文本行 `GW,SCAN`、`GW,READY`、`GW,BUSY`、`GW,PROV,addr`、`GW,NODE_READY,addr`
