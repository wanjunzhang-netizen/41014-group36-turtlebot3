# GitHub 代码与项目网页上传指南

项目根目录：`D:\机器人1\final_work_test\github_project`。代码在 main 分支；网页源文件在 website/；把该目录发布到 github_page 分支。无需切换分支或删除仿真代码。

## 1. 在 VS Code 打开项目

File → Open Folder → 选择上述 github_project。打开 README.md。按 Ctrl+Shift+V 预览说明。Terminal → New Terminal 打开 PowerShell。

```powershell
Set-Location 'D:\机器人1\final_work_test\github_project'
git --version
```

本地预览网页（不要直接双击 HTML，因为结果表需要通过 HTTP 读取 JSON）：

```powershell
.\.venv\Scripts\python.exe -m http.server 8765 --bind 127.0.0.1 --directory website
```

在你电脑的浏览器打开 http://127.0.0.1:8765 。结束预览在该终端按 Ctrl+C。

## 2. 创建空仓库

登录 https://github.com ，右上角 + → New repository。

- Repository name：`41014-group36-turtlebot3`。
- Description：`Group 36 — Single-camera TurtleBot3 visual docking in MuJoCo`。
- 选择 Public，让老师无需登录即可查看代码和网页。
- 不勾选 Add README，不添加 .gitignore 或 license（本地已有文件，第三方资产保留其许可证）。
- 点击 Create repository，复制 HTTPS 地址。

## 3. 填写项目链接

在 VS Code 左侧展开 website，打开 config.js。把 repository 的空字符串改成真实地址，例如：

```javascript
window.PROJECT_CONFIG = {
  repository: "https://github.com/你的用户名/41014-group36-turtlebot3",
  videoFile: "",
  youtubeId: ""
};
```

把 README.md 顶部 Code repository、Project website、Demo video 的 TODO 换成真实链接。网页地址一般是 `https://你的用户名.github.io/41014-group36-turtlebot3/`，最终以 Settings → Pages 给出的地址为准。视频未录完可以先保留待完成说明，但提交作业前必须补齐。Ctrl+S 保存。

## 4. 上传代码

下面的初始化命令用于当前尚未建立 Git 仓库的项目。如果已有仓库，不要重复添加 origin；先用 git status 和 git remote -v 查看。

```powershell
git init -b main
git config user.name "wanjun zhang"
git config user.email "替换成你的GitHub提交邮箱或noreply邮箱"
git add .
git status --short
```

检查列表中没有 .venv、缓存、私人文件和录屏原始素材。项目内的模型、代码、校准结果、evidence 和 website 应上传。确认后：

```powershell
git commit -m "Add single-camera docking simulation and project page"
git remote add origin https://github.com/你的用户名/41014-group36-turtlebot3.git
git push -u origin main
```

登录提示出现时，在浏览器完成自己的 GitHub 登录。不要把密码或 token 写进代码。若提示 origin already exists，先检查 git remote -v 是否为目标仓库，不要随意覆盖。若远程非空导致拒绝推送，不要强制推送，先解决远程历史问题。

## 5. 发布独立网页

在项目根目录运行：

```powershell
git subtree push --prefix website origin github_page
```

这个命令只将 website 目录发布为 github_page 分支根目录，工作目录仍是 main。然后到 GitHub 仓库：

1. Settings → 左侧 Pages。
2. Build and deployment → Source 选择 Deploy from a branch。
3. Branch 选择 github_page；文件夹选择 / (root)。
4. Save。等部署完成，点击 Visit site。
5. 回到仓库 Code 页，右侧 About 的齿轮，把该网页地址填入 Website，保存。

注意：选择 github_page 分支的根目录，不是 main 的根目录。当前网页首页在 website/index.html，subtree 发布后变成网页分支根目录的 index.html。

## 6. 后续更新

先在本地修改、保存和验证，再运行：

```powershell
git add .
git commit -m "Update project page and demonstration video"
git push origin main
git subtree push --prefix website origin github_page
```

代码功能改动使用 feature/xxx 分支，验证后通过 Pull Request 合并 main。你是单人小组，可以记录自查，不能虚构队友审查。网页发布命令在包含最新 website 的 main 上执行。

## 7. 提交前验证

- 用未登录窗口打开仓库和项目页，确认无需权限。
- 网页显示组号36、姓名、学号、代码链接、模型、坐标图、参数、定量结果、贡献。
- 点击视频，能播放、有声音/字幕、总时长不超过5分钟。
- 结果表显示10条数据；图片正常；代码链接指向自己的仓库。
- 从 GitHub Code → Download ZIP 解压到一个新目录，按 README 新建环境、安装依赖、运行 seed 0。不要复制旧 .venv。这一步才检验上传资产是否齐全。

官网参考：https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site
