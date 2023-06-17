# PATCHWORK

使用 `Rust` 重写 [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus)。当前处于开发中。

## 组织结构

* `example`: 目录包含一些用例，当前主要作用是用于验证 `cpp` 与 `rust` 两边的输出日志是否一致。
* `src`: 项目源代码。
* `*.log`: 临时的日志文件，以后整理吧。每一组文件对应 `example` 中的一个程序。

## TODO

* [x] 函数 `Patchwork::estimate_ground` 中的 `concentric_idx` 变量会越界，并且 `cpp` 的实现中可以得到精确但奇怪的值，~~具体参见 [!23](https://github.com/url-kaist/patchwork-plusplus/issues/23)~~ 已解决 [!28](https://github.com/url-kaist/patchwork-plusplus/pull/28/files)。

* [ ] `Eigen` 与 `nalgebra` 两个库的 `svd` 计算结果不同，虽然可以验证二者的结果均正确，但由此导致的后续数据均有一定的不同，影响验证重写的正确性

* [ ] 构建动态链接库并导出 `c api` 提供给 `cpp` 调用，通过可视化验证。
