## 改了什么

<!-- 一句话说清；涉及多个关注点请拆成多个 PR -->

## 为什么改

<!-- 解决什么问题；关联 issue 用 #编号 -->

## 怎么验证

- [ ] 全量构建通过：`cmake -B build -S . && cmake --build build -j"$(nproc)"`
      结果：__ error / __ warning（告警是否来自 vendored 第三方代码：是/否）
- [ ] `clang-format` 已应用于改动的 `.cpp/.hpp`
- [ ] 相关测试程序已运行（列出程序名与结果）
- [ ] 实机验证（云台/相机/下位机相关改动必填）：已上机 / 未上机

## 与上游 sp_vision_25 的关系

- [ ] 本次未触碰上游文件
- [ ] 本次修改了上游文件，已在 `NOTICE` 第 3 节改动清单追加一行
- [ ] 本次引入了新的第三方代码/权重，已在 `THIRD_PARTY_NOTICES.md` 登记并确认许可兼容

## 文档

- [ ] 本次改动改变了函数作用 / 新增或删除函数 → 已更新 `doc/overview.md`、`doc/directory_tree.md`、`doc/code/**`
- [ ] 本次解决了一个新问题 → 已在 `doc/troubleshooting.md` 追加条目并同步快速索引
- [ ] 无需更新文档（仅实现细节调整，语义未变）

## 只提交源码自检

- [ ] 暂存区没有二进制、模型权重、数据集、厂商 SDK 库、日志/录制数据
      （`git diff --cached --name-only` 已逐条看过）
