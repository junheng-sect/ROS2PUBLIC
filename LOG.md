# 开发日志

## 问题记录

- 2026-03-06 | 问题：提供 GitHub 凭据并要求继续完成远程上传。
  解答：已配置仓库本地身份（`user.name=junheng-sect`、`user.email=junhengsect+1647307223@qq.com`），成功将 `ubuntu2204simple` 推送到远程；并将本地跟踪关系恢复为 `origin/ubuntu2204simple` 的标准配置（不保留令牌 URL）。
- 2026-03-06 | 问题：继续完成本地 Git 操作，并将工作空间上传到新建分支 `ubuntu2204simple`。
  解答：已在 `zjh_ws` 初始化本地 Git 仓库、配置撤回别名（`undo/unstage/discard/last/rollback`）、绑定远程 `origin`、创建并提交 `ubuntu2204simple` 分支；推送到 `origin/ubuntu2204simple` 时因本机缺少 GitHub 凭据失败（HTTPS 无用户名凭据，SSH 无公钥授权）。
- 2026-03-06 | 问题：阅读 `AGENTS.md` 与 `LOG.md`，明确协作要求并熟悉接下来要做的功能。
  解答：已完成两份文档核对；后续将严格执行记录、构建、launch 验证、中文详细注释、`README.md` 命名与分支策略，并以 `tvec`（视觉检测/位姿发布）与 `debug_interface`（调试消息接口）为当前功能基础继续开发。
- 2026-03-05 | 问题：将 `zjh_ws` 接入 GitHub 远程并创建 `simple` 分支上传源码。
  解答：已在本工作空间初始化 Git、配置撤回别名、接入远程仓库并推送 `simple` 分支；提交中排除了运行日志目录。
- 2026-03-05 | 问题：远程 `simple` 已存在，是否覆盖远程分支？
  解答：已按确认执行强制推送，使用本地 `zjh_ws` 的 `simple` 覆盖远程 `origin/simple`。
- 2026-03-05 | 问题：将 `tcec_test` 功能包改名为 `tvec` 并构建。
  解答：已完成目录、包名、模块名、launch/rviz/readme 的全量重命名（`tcec_test -> tvec`），并通过构建与启动验证。
- 2026-03-05 | 问题：把 `source install/setup.bash` 加入 `.bashrc`。
  解答：已将 `.bashrc` 中工作空间 source 行切换为 `zjh_ws`，并使用存在性判断避免报错：`if [ -f ~/project/zjh_ws/install/setup.bash ]; then source ...; fi`。
- 2026-03-05 | 问题：不要再生成小写 `readme.md`，并删除已有小写文档。
  解答：已将规则写入“协作与记录要求”（仅允许 `README.md`），并删除现有小写 `readme.md` 文件。
- 2026-03-05 | 问题：把当前工作空间内容上传到远程仓库。
  解答：已在 `zjh_ws` 的 `simple` 分支完成提交并推送到 `origin/simple`。

## 修改记录

### 工作空间与协作规范
- 2026-03-06：按要求通读 `AGENTS.md` 与 `LOG.md`，完成规则确认与后续开发重点梳理，并将本轮问答写入日志。
- 2026-03-05：进入新工作空间 `~/project/zjh_ws` 后，保留规则并重置历史记录内容（保留标题）。
- 2026-03-05：完成本工作空间 Git 初始化与本地撤回别名配置（`undo/unstage/discard/last`）。
- 2026-03-05：更新 `~/.bashrc` 自动加载当前工作空间环境：由 `project_ws` 改为 `zjh_ws` 的 `install/setup.bash`（带文件存在判断）。
- 2026-03-05：新增文档规范：功能包文档仅使用大写 `README.md`，禁止创建小写 `readme.md`。
- 2026-03-05：确认当前开发工作空间为 `~/project/zjh_ws`，并将其固化为默认工作路径。
- 2026-03-05：新增代码规范：后续代码需编写详细注释，重点解释关键变量、计算流程与控制逻辑。
- 2026-03-05：新增注释语言规范：后续代码注释统一使用中文。
- 2026-03-05：日志结构调整：`### 功能包修改记录` 改为按功能包小标题分组记录，并在该节加入固定记录要求。

### Git 仓库与远程
- 2026-03-06：基于用户提供凭据完成 `ubuntu2204simple` 分支远程推送；随后执行 `git fetch` 并将本地上游分支规范化为 `origin/ubuntu2204simple`。
- 2026-03-06：在 `~/project/zjh_ws` 新建本地 Git 仓库，新增撤回别名：`undo/unstage/discard/last/rollback`。
- 2026-03-06：重新绑定远程仓库 `origin=https://github.com/junheng-sect/ROS2PUBLIC.git`，目标上传分支为 `ubuntu2204simple`。
- 2026-03-06：创建本地分支 `ubuntu2204simple` 并完成提交 `518b804`；远程推送受认证限制未完成（需配置 GitHub HTTPS 凭据或 SSH Key）。
- 2026-03-05：接入远程仓库 `origin=https://github.com/junheng-sect/ROS2PUBLIC.git`。
- 2026-03-05：创建并推送 `simple` 分支，上传本工作空间源码。
- 2026-03-05：执行 `git push -f origin simple`，远程 `simple` 已由 `71fdeb0` 强制更新覆盖。
- 2026-03-05：执行常规推送，提交 `426eaaf` 已上传到 `origin/simple`。

### 功能包修改记录
- 记录要求：本节必须按“功能包小标题”分组记录修改，后续新增记录不得混写在同一列表中。

#### tvec
- 2026-03-05：将功能包 `tcec_test` 全量重命名为 `tvec`：目录 `src/tvec`、Python 包 `tvec`、资源索引、`package.xml/setup.py/setup.cfg`、launch 文件 `tvec.launch.py`、RViz 配置 `tvec.rviz`、话题前缀 `/tvec/...`、README/readme 与默认 CSV 路径均已同步更新。
- 2026-03-05：完成 `tvec` 验证：`colcon build --packages-select tvec`、`source install/setup.bash`、`ros2 launch tvec tvec.launch.py world_name:=rover` 启动成功（桥接、节点、RViz 正常拉起）。
- 2026-03-05：按新文档规范删除小写文件 `src/tvec/readme.md`，仅保留 `src/tvec/README.md`。
- 2026-03-05：按“详细注释”要求补充现有代码注释：`src/tvec/tvec/tvec_rvec_node.py` 增加参数语义、检测与位姿估计流程、CSV记录与发布流程注释；`src/tvec/launch/tvec.launch.py` 增加世界切换与桥接逻辑注释；并完成 `colcon build` 与 `source install/setup.bash`。
- 2026-03-05：将现有英文注释统一改为中文：覆盖 `src/tvec/tvec/tvec_rvec_node.py`、`src/tvec/launch/tvec.launch.py`；完成 `colcon build`、`source install/setup.bash`，并执行 `ros2 launch tvec tvec.launch.py` 启动验证后停止。
- 2026-03-05：为 `tvec` 新增调试话题发布：在 `src/tvec/tvec/tvec_rvec_node.py` 增加 `debug_interface/msg/TVecRVec` 发布器，实时发布到 `/debug/tvec`；更新 `src/tvec/package.xml` 增加 `debug_interface` 依赖。已完成 `colcon build`、`source install/setup.bash`，并通过 `ros2 topic info /debug/tvec` 验证话题类型为 `debug_interface/msg/TVecRVec`。

#### debug_interface
- 2026-03-05：补齐 `debug_interface` 接口包实现：新增 `msg/TVecRVec.msg`（`header+tvec+rvec`），并更新 `CMakeLists.txt/package.xml` 的 `rosidl` 生成配置；重新构建后通过 `ros2 interface show debug_interface/msg/TVecRVec` 测试。
- 2026-03-05：将 `src/debug_interface/msg/TVecRVec.msg` 的字段注释统一改为中文（时间戳/参考系、tvec 语义、rvec 语义）。
