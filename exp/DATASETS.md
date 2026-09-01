# 本地数据集清单

更新时间：`2026-08-13T10:06:15Z`

统一根目录：`/mnt/zsn/lyb/arti-skill/exp`

`Artiverse` 已按要求跳过审计，原目录
`/mnt/zsn/lyb/arti-skill/exp/artiverse` 未改动。其余四套数据均已放到统一根目录。

## 直接使用路径

| 数据集 | 状态 | 数据包根目录 | 直接使用的 URDF 根目录 |
|---|---|---|---|
| Articraft-10K | `VERIFIED_COMPLETE` | `exp/Articraft-10K` | `exp/Articraft-10K/released_urdf` |
| LAM released outputs | `VERIFIED_RELEASE_COMPLETE` | `exp/Articulated-Object-Code` | `exp/Articulated-Object-Code/released_outputs`，按 `manifest.csv` 的 `tier` 和 `rel_path` 选择 |
| PartNet-Mobility | `LOCAL_COMPLETE_PROVENANCE_LIMITED` | `exp/PartNet-Mobility` | `exp/PartNet-Mobility/data/dataset` |
| PhysX-Mobility | `VERIFIED_OFFICIAL_ARCHIVE` | `exp/PhysX-Mobility` | `exp/PhysX-Mobility/extracted/PhysX_mobility/urdf` |

所有路径均相对于 `/mnt/zsn/lyb/arti-skill`。机器可读的精确版本、哈希和计数见
`exp/dataset_inventory.json`。

## 验收摘要

### Articraft-10K

- 来源固定为 Hugging Face `camvsl/Articraft-10K@3c79d5a05bb7cb6bf7bfee5e090176636ee3ac65`。
- 下载到 `9,996/9,996` 个 `rec_*.tar.gz`，总计 `726,461,092` bytes；逐文件 LFS SHA-256 全部匹配固定版本。
- 安全解包后得到 `9,996` 个对象目录和 `9,996` 个 `model.urdf`；XML 解析失败 `0`，缺失相对 mesh 引用 `0`。
- `exp/baselines/Articraft-10K-official` 是另一份 GitHub source-record checkout：有
  `model.py`/`record.json`，没有 URDF，且记录数与 HF 发布版不同。它没有被合并或冒充为本发布版。

### LAM released outputs

- 来源固定为 Hugging Face
  `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0`。
- 官方当前发布的 `viable.tar.gz`、`loads_only.tar.gz`、`broken.tar.gz` 均已下载并通过 LFS SHA-256。
- `manifest.csv` 共 `3,217` 个发布对象：`viable=2,533`、`loads_only=299`、`broken=385`；清单中的
  `3,217/3,217` 个 `generated.urdf` 均存在且 XML 可解析。
- `viable` 和 `loads_only` 层缺失相对 mesh 引用均为 `0`。`broken` 是上游明确保留的失败层，其中
  `81` 个对象合计有 `354` 个缺失相对 mesh 引用；不要把它作为可用层。
- 解包树中还有 pipeline/intermediate URDF 副本，因此递归数到的 `4,294` 个 `generated.urdf`
  不是发布对象数；正式分母必须以 `manifest.csv` 为准。
- LAM 代码位于 `exp/baselines/LAM-official`，固定到 Git commit
  `0b3a87beb8c35273a5acf8681221791aff746d8e`。

### PartNet-Mobility

- 本地抽取树有 `2,347` 个对象；每个对象均有一个 `mobility.urdf`、`meta.json` 和
  `semantics.txt`。
- 原始包 `partnet-mobility-v0.zip` 为 `3,268,124,298` bytes，SHA-256 为
  `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff`，ZIP 完整性测试通过。
- Hugging Face `sapien-sim/PartNetMobility` 固定版本
  `ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f` 是人工审批 gated 数据。当前账号没有下载授权，
  因而无法把本地每个对象与该 HF 快照做字节级认证。这里可以确认“本地集合完整”，不能声称
  “本地字节已由 HF 固定版本认证”。

### PhysX-Mobility

- 来源固定为 Hugging Face
  `Caoza/PhysX-Mobility@d0768ee9e1415f6be8db78d6389ba018b85134c0`。
- `PhysX-Mobility.zip` 为 `937,374,668` bytes，SHA-256 为
  `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`；与官方 LFS 对象一致，
  ZIP 完整性测试通过。
- 抽取树与包内闭包一致：`2,024` 个最终 URDF、`2,024` 个 `finaljson` 和 `2,024` 个
  `partseg` 对象目录。这里已经有转换后的 URDF，不需要为读取数据再运行转换。
- 该集合是 PartNet-Mobility 的 `2,024` ID 子集；PartNet 另有 `323` 个对象。
- `exp/baselines/physx_mobility/official_repo` 记录了 PhysX-Anything remote 和 commit
  `e221826e6176d940905126d1894f9c1c933b70a8`，但当前 checkout 的 tracked 工作树为空，不能当作
  可运行的转换代码目录。

## 兼容路径

为避免已有脚本失效，原路径保留为符号链接：

- `/mnt/zsn/lyb/PartNet_Mobility` -> `exp/PartNet-Mobility`
- `/mnt/zsn/lyb/PhysX-Mobility-official` -> `exp/PhysX-Mobility`
- `exp/baselines/LAM-official-dataset` -> `exp/Articulated-Object-Code`
- `.cache/table6_sources/lam/dataset` -> `exp/Articulated-Object-Code`

不要使用 Ctrl-3D 下的 PhysX 副本作为原始数据源：它包含额外生成的
`*_collision.urdf`/`*_sim.urdf`，不是官方原始包的精确抽取闭包。
