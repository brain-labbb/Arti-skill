# physics_caption

56 个类别，每个类别 5 个 seed，共 280 个可复现的 Articraft 导出。

目录结构：

```text
<category_slug>/seed_<n>/
├── model.py
├── model.urdf
└── assets/meshes/       # 模板需要时生成的 OBJ mesh
```

`model.urdf` 中使用 primitive geometry 的 seed 不会额外生成 mesh 文件；其 URDF 本身仍是完整可加载的模型。所有 seed 均已生成 URDF；其中需要外部 mesh 的 seed 已保留在 `assets/meshes/`，且未保留 JSON 报告或清单。
