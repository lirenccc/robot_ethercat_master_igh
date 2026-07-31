# CoolDrive JMDT（IgH）

IgH 在 `EtherCATServo` 内按 `PdoLayout::COOLDRIVE_JMDT` 运行时配置 PDO（无需 ENI）。

型号 ID：`COOLDRIVE-JMDT`（VID `0x00000748` / PID `0x00000019`）。

双网口：内核 IgH 主站 0/1 对应左右网卡；产品层 `ETHERCAT_MASTER_INDEX` + `/right` `/left` 命名空间。
