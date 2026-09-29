# rm_typec 设备树 `spi-max-frequency` 提示诊断

## 当前状态与结论

- `boards/rm_typec/rm_typec.dts` 中的 `&spi1` 下有两个 BMI08x 子节点，分别在第 286、300 行写了 `spi-max-frequency = <DT_FREQ_M(8)>;`。
- 两个子节点的 `compatible` 分别是 `bosch,bmi08x-accel` 和 `bosch,bmi08x-gyro`；`reg = <0>` 和 `reg = <1>` 对应 `&spi1` 中两项 `cs-gpios`。
- 当前工作区旁的 Zephyr 目录中，`dts/bindings/sensor/bosch,bmi08x-accel-spi.yaml` 与 `bosch,bmi08x-gyro-spi.yaml` 均包含 `spi-device.yaml`。该公共 binding 定义 `spi-max-frequency` 为必需的整数属性，单位 Hz。
- 因此，单凭“Property name is not defined in the type binding”这条编辑器提示，不能认定这两行 DTS 写错。更可能是 DTS 语言服务没有识别 SPI 父节点上下文，选择了通用 `bosch,bmi08x-*.yaml`，或加载了另一套 binding。此原因尚未通过编辑器日志确认。

## 为什么总线位置影响 binding

```text
&spi1  (SPI 控制器)
├── bmi08x@0  compatible = "bosch,bmi08x-accel"
└── bmi08x@1  compatible = "bosch,bmi08x-gyro"
                  ↓
Zephyr 根据 compatible 和父节点的 SPI 总线类型选择 *-spi.yaml
                  ↓
*-spi.yaml 包含 spi-device.yaml，其中要求 spi-max-frequency
```

相同的 `compatible` 也可能用于 I²C，因此只看 `compatible` 而忽略父节点，工具就可能选错 binding。`DT_FREQ_M(8)` 在预处理后表示 8,000,000 Hz，数值形式符合属性类型。

## 手工排查顺序

1. 在编辑器中查看这两个节点实际关联到哪个 binding。如果指向 `bosch,bmi08x-accel.yaml`、`bosch,bmi08x-gyro.yaml` 或 I²C 版本，而不是对应的 `*-spi.yaml`，问题在绑定解析或项目上下文。
2. 确认编辑器把 `/home/shm-white/skywalker-zephyr/zephyr` 当作当前 Zephyr 源码目录，并识别 `boards/rm_typec/rm_typec.dts` 是 `&spi1` 的板级 DTS。项目推荐的 DTS 扩展见 `.vscode/extensions.json`。重新打开工作区或重启 DTS 语言服务后再次查看诊断。
3. 若仍有疑问，由用户在正确的 west 工作区执行一次板级构建，例如：`west build -b rm_typec/stm32f407xx samples/hello -d /tmp/rm_typec-hello-check -p always -- -DBOARD_ROOT=/home/shm-white/skywalker-zephyr/skywalker_code`。这里将输出放在 `/tmp`，避免覆盖已有构建产物。若构建能完成设备树处理而编辑器仍报此属性，说明它是编辑器侧的误报；若构建失败，依据首条实际错误继续定位，不要仅凭本条编辑器提示删掉属性。
4. 可检查构建生成的 `zephyr/zephyr.dts` 与设备树处理日志，确认两个节点仍位于 `spi1` 下，并确认所使用的 Zephyr 源码及 binding 路径。若编译器报告缺少 `spi-max-frequency`，要检查它是否选中了另一个 binding 或 overlay 是否改变了父子关系。

## 自检与边界

- 不要将 `spi-max-frequency` 移到 `&spi1` 控制器节点。它描述每个 SPI 外设允许的最高时钟，两个子节点各自需要。
- `boards/damiao/dm_mc02/dm_mc02.dts` 也以同样方式给 BMI08x 的 SPI 子节点设置该属性，可作本仓库内参考。
- 尚未执行构建或读取编辑器语言服务日志，所以“编辑器误报”的具体触发原因仍待验证。
- [ ] 编辑器选中 `*-spi.yaml`；[ ] 构建完成设备树处理；[ ] 两个节点各保留自己的频率属性。
