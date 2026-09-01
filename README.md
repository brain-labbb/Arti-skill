# arti-skill pipeline

这个仓库只负责把完整流程组织起来。核心实现与资产数据保留为两个独立仓库，并由顶层 Git commit 固定到精确版本：

- `arti-template/`：模板、SDK、viewer 与模板测试；
- `articraft_data/`：记录、图片、数据工具与 LFS 对象；
- 顶层：启动脚本、配置、环境示例、评测 pilot 和端到端检查。

## 首次检出

```bash
git clone --recurse-submodules <pipeline-repo-url> arti-skill
cd arti-skill
cp .env.example .env
just bootstrap
just doctor
just test-e2e
```

`bootstrap` 默认只检出两个 submodule，不下载大体积 LFS 内容，也不安装两个项目的依赖。需要数据时，在 `.env` 中设置：

```dotenv
ARTI_PULL_LFS=1
ARTI_LFS_INCLUDE=data/records/**,picture/**
```

需要同时创建子项目环境时，可再设置 `ARTI_SETUP_TEMPLATE=1` 和/或 `ARTI_SETUP_DATA=1`。这两个操作要求系统中已有 `just` 与 `uv`。

## 常用命令

```bash
just bootstrap        # 初始化 submodule；LFS 和依赖按 .env 开关处理
just doctor           # 检查工具、gitlink 与工作树布局
just test-e2e         # 快速验证 fresh-checkout 拓扑
just setup-template   # 安装模板仓库依赖
just setup-data       # 安装数据仓库依赖
just test-template    # 模板仓库 smoke tests
just test-data        # 数据仓库 smoke tests
just eval metrics     # 运行 eval_pilot 子命令
just viewer           # 启动模板 viewer
```

也可以直接运行 `scripts/` 下的脚本；它们都会以本文件所在仓库为根目录，不依赖调用时的当前目录。

## Submodule 发布约束

顶层只记录两个子仓库的 commit SHA。合并或发布顶层 commit 前，必须先把对应子仓库 commit（以及数据仓库的 LFS 对象）推送到团队可访问的远端，再提交顶层 gitlink。`.gitmodules` 当前使用同一托管命名空间中的相对地址；如果远端仓库名不同，应在首次发布前更新这两个 URL。

官方 Articraft-10K 记录不再捆绑在模板仓库中；需要公开数据时应从其独立数据源获取。模板侧保留下来的本地记录已同步进数据仓库，避免只存在于忽略目录中。
