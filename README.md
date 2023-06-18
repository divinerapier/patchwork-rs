# PATCHWORK

使用 `Rust` 重写 [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus)。当前处于开发中。

## 组织结构

* `example`: 目录包含一些用例，当前主要作用是用于验证 `cpp` 与 `rust` 两边的输出日志是否一致。
* `src`: 项目源代码。
* `*.log`: 临时的日志文件，以后整理吧。每一组文件对应 `example` 中的一个程序。

## Usage

### Patchwork

可参考 [example](./examples/readbin.rs)。

### 初始化矩阵

使用如下函数初始化矩阵

* `matrix_from_raw_buffer`
* `matrix_from_raw_buffer_drop_last`
* `nx3f32_from_raw_buffer`
* `nx4f32_from_raw_buffer`

#### 函数 matrix_from_raw_buffer

根据 `Buffer` 的数据初始化 (m x N) 维矩阵。

函数签名为:

``` rust
pub fn matrix_from_raw_buffer<const N: usize>(
    mut buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<N>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
>
```

> N: 常量泛型参数。指定目标矩阵列的数量。
> buffer: 字节数组。

#### 函数 matrix_from_raw_buffer_drop_last

根据 `Buffer` 的数据初始化 (m x N) 维矩阵。并将最后一列数据丢弃。

例如: 当 `N=3` 时，会将 `Buffer` 按照列为 **`4`** 的矩阵进行解析，但是仅保留前 **`3`** 列的数据。

因为，点云数据的信息中，每个点有 `4` 个维度的信息。在一些计算中，仅希望保留前 `3` 个维度。

函数签名为:

``` rust
pub fn matrix_from_raw_buffer_drop_last<const N: usize>(
    mut buffer: Vec<u8>,
) -> nalgebra::Matrix<
    f32,
    nalgebra::Dyn,
    nalgebra::Const<N>,
    nalgebra::VecStorage<f32, nalgebra::Dyn, nalgebra::Const<N>>,
> 
```

> N: 常量泛型参数。指定目标矩阵列的数量。
> buffer: 字节数组。

#### 函数 nx{M}f32_from_raw_buffer

对函数 `matrix_from_raw_buffer` 的简单封装。

## TODO

* [x] 函数 `Patchwork::estimate_ground` 中的 `concentric_idx` 变量会越界，并且 `cpp` 的实现中可以得到精确但奇怪的值，~~具体参见 [!23](https://github.com/url-kaist/patchwork-plusplus/issues/23)~~ 已解决 [!28](https://github.com/url-kaist/patchwork-plusplus/pull/28/files)。

* [x] `Eigen` 与 `nalgebra` 两个库的 `svd` 计算结果不同，虽然可以验证二者的结果均正确，但由此导致的后续数据均有一定的不同，影响验证重写的正确性

* [ ] 构建动态链接库并导出 `c api` 提供给 `cpp` 调用，通过可视化验证。
