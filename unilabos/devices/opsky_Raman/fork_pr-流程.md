好的，下面是完整总结的 Git 从 Fork 到 PR 的标准操作流程，适用于今后任何你想向别人仓库提交 `.py` 或其他文件的场景：

## ✅ Git 从 Fork 到 PR 提交流程（以 `zyb_fork.py` 为例）

------

## 🧩 第一步：Fork 原始仓库

1. 打开原始仓库页面：
    👉 `https://github.com/taipinghu/program_scripts`
2. 点击右上角 `Fork` 按钮
3. GitHub 会创建你自己的 Fork，地址类似：
    👉 `https://github.com/ZYB10/zyb_program_scripts`

------

## 🧩 第二步：Clone 到本地或远程环境

```bash
git clone https://github.com/ZYB10/zyb_program_scripts.git
cd zyb_program_scripts
```

------

## 🧩 第三步：配置 Git 用户（首次使用时必须）

```bash
git config --global user.name "你的名字"
git config --global user.email "你的 GitHub 邮箱"
```

例如：

```bash
git config --global user.name "Yubing Zhao"
git config --global user.email "zyb123456@gmail.com"
```

------

## 🧩 第四步：创建一个新的功能分支

```bash
git checkout -b zyb/feature_script_upload
```

------

## 🧩 第五步：创建并编辑你的文件

例如创建一个简单脚本：

```bash
echo 'print("hello world")' > zyb_fork.py
```

或者用 VS Code 手动创建并编辑保存。

------

## 🧩 第六步：添加、提交、推送代码

```bash
git add zyb_fork.py
git commit -m "Add zyb_fork.py with hello world"
git push origin zyb/feature_script_upload
```

------

## 🧩 第七步：在 GitHub 上发起 Pull Request（PR）

1. 打开你的仓库：
    👉 `https://github.com/ZYB10/zyb_program_scripts`
2. 点击 “Compare & pull request” 或手动点：
   - `Pull Requests` → `New pull request`
   - base repository: `taipinghu/program_scripts`
   - base branch: `develop`
   - compare: `zyb/feature_script_upload`
3. 填写标题和说明后提交 PR

------

## 🧩 可选补充：查看和同步 upstream（原仓库）更新

（当原仓库更新了，你可以这样同步）

```bash
git remote add upstream https://github.com/taipinghu/program_scripts.git
git fetch upstream
git checkout develop
git merge upstream/develop
```

------

## ✅ 最终效果：

你通过一个新的分支提交了代码，并向原始项目的 `develop` 分支发起了 PR，整个流程干净、可追踪、利于协作。

------

如你今后还要多次提 PR，可以把这套流程保存在你 VS Code 或 Typora 里当作个人开发模板。是否需要我做成一张 Markdown 思维导图？