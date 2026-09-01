# NURBGen CAD-numeric supplementary

状态：`NOT_RUN_GATED_MODEL`。这是独立的 **NURBGen** supplementary，不是 Text2CAD 复现，也不使用 Text2CAD 名称或结果。

## 固定来源

- 作者源码：`https://github.com/SadilKhan/NURBGen`，commit `62855d4b258082e5fbd220badf056618f7840939`。
- 作者 LoRA：`https://huggingface.co/SadilKhan/NURBGen`，revision `f2f88e264e735353506a853e761e96d8545649d9`。
- LoRA 文件：`adapter_model.safetensors`，528,550,256 bytes；HF tree git blob OID `42306119def1dca94294acd148d6d8aa44063c43`；HF security record 所示文件 SHA-256 `d381fa4b5e82c1d4602e4019b5e444d3208ede942c9de841e22d72c793873d54`，本地未下载，故未做本地 hash 复核。
- base：`Qwen/Qwen3-4B` revision `1cfa9a7208912126459214e8b04321603b3df60c`。三个权重 shard 的 LFS SHA-256 已固定在 `provenance.json`。
- base LICENSE：Apache-2.0，本地证据 SHA-256 `832dd9e00a68dd83b3c3fb9f5588dad7dcf337a0db50f7d9483f310cd292e92e`。
- NURBGen 模型卡声明 `apache-2.0`；固定 GitHub 源码树没有 LICENSE 文件，因此不能把模型卡许可无条件外推为源码许可。

## 阻塞证据

2026-08-11 对固定 revision 的 `adapter_config.json` 做匿名官方下载请求，HTTP 返回 `401`、`X-Error-Code: GatedRepo`，正文要求先获得访问权并认证。当前进程没有 `HF_TOKEN` 或 `HUGGING_FACE_HUB_TOKEN`。原始响应保存在：

- `/mnt/zsn/lyb/NURBGen/hf_adapter_headers.txt`
- `/mnt/zsn/lyb/NURBGen/hf_adapter_response.txt`

因此没有模型可用于真实生成。没有启动非 benchmark smoke，没有消费 18 个 benchmark attempt，也没有运行 scorer。缺模型是实验未运行，不是方法失败或 0 分。

## 本轮结果

| 项目 | 结果 |
|---|---:|
| 非 benchmark official smoke | N/R (`NOT_RUN`) |
| 18 prompt generation | N/R (`0/18 attempted`) |
| CAD numeric, 20 constraints | N/R (`NOT_RUN`) |
| Count | N/R (`NOT_RUN`; 运行时该列在 `cad_numeric` panel 为 N/A) |
| scorer repeat 1 / 2 | N/R / N/R |

冻结 prompt 文件 SHA-256 已核验为 `0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e`。未修改冻结 protocol/spec/prompt，也未修改 `exp/Nano3dresults.md`。

## 可执行路径

`run_nurbgen_constraints.py` 只接受工作区内的本地 base 与 adapter，强制 `CUDA_VISIBLE_DEVICES=3`，并在模型文件 hash、源码 commit 与 prompt hash 全部匹配后执行。它先运行作者 README 的非 benchmark smoke；smoke 成功后才逐题进行 18 个唯一 attempt。每题固定 `max_new_tokens=8192`、作者 HuggingFace/PEFT 示例的 `do_sample=False`、1800 秒、repair 0，保存 raw response/NURBS JSON、STEP、GLB、失败和耗时；仅做固定的 STEP/STL mm 到 GLB m 的 `0.001` 单位变换，不按目标尺寸事后缩放。最后使用 `cad_numeric` panel 连续跑两次冻结 scorer 并要求 summary 完全一致。

```bash
CUDA_VISIBLE_DEVICES=3 /path/to/nurbgen-python \
  exp/baselines/nurbgen/run_nurbgen_constraints.py \
  --base-dir /mnt/zsn/lyb/path/to/pinned-Qwen3-4B \
  --adapter-dir /mnt/zsn/lyb/path/to/pinned-NURBGen-LoRA \
  --output-dir /mnt/zsn/lyb/arti-skill/exp/runtime/nurbgen_table4_constraints_v2
```

必须先在作者 Hugging Face 页面完成 gate 授权；本 runner 不绕过许可、不从非作者镜像获取 LoRA。
