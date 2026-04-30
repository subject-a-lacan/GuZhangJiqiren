如果你也在用 Windows 玩 Claude Code，并且苦恼于「为什么它看不到我本地文件夹里的图片」，这篇文章就是为你写的。

先说结论
进入你的项目文件夹，在命令行运行：

bash


claude mcp add --scope project image-viewer -- cmd /c npx -y inchat-image-viewer-mcp
然后启动 Claude Code，输入 /mcp，看到绿色的 ✓ Connected 就搞定了。

接下来讲讲我是怎么一步步踩坑踩过来的。

问题：Claude Code 看不了图片
事情的起因很简单。我在写博客的时候，需要 Claude Code 帮我查看项目里的截图，确认有没有隐私信息。于是我让它去读图片文件。

结果：

Claude Code 读取图片返回空
返回空。什么都没有。

Claude Code 自带的 Read 工具只能读文本文件，遇到图片这种二进制文件就直接返回空。不管你怎么换路径、换格式，结果都一样——它就是个「瞎子」。

找解法：问 Perplexity
我把这个问题丢给了 Perplexity（一个 AI 搜索引擎），让它帮我找解决方案。

和 Perplexity 的对话
Perplexity 告诉我，需要给 Claude Code 装一个 MCP（Model Context Protocol）工具。MCP 是一种让 AI 调用外部工具的协议，相当于给 Claude Code 装「插件」。它推荐了一个叫 inchat-image-viewer-mcp 的工具，专门用来查看图片。

安装命令：

bash


claude mcp add image-viewer -- npx -y inchat-image-viewer-mcp
看起来很简单，复制粘贴就行了。

第一次踩坑：在 C 盘运行命令
我当时随手在 C 盘的命令行里运行了这条命令，系统提示 Added stdio MCP server，看起来很完美。

然后我切到项目文件夹，启动 Claude Code，输入 /mcp：

MCP 未配置
No MCP servers configured。 什么都没有。

我在 C 盘装的配置，写进了全局配置文件里。但 Claude Code 在项目里启动时，默认只读当前项目的配置，所以在项目里根本看不到。

第二次尝试：在项目目录下安装
继续和 Perplexity 排查
我把问题反馈给 Perplexity，它让我：

先删掉之前在 C 盘装的：claude mcp remove image-viewer
进入项目文件夹，用 --scope project 参数重新安装
但这里还有一个 Windows 特有的坑：npx 在 Windows 下不是一个真正的 .exe 程序，而是一个 .cmd 脚本。如果让 Claude 在后台直接调起 npx，进程会崩溃。

解决方法是在前面加 cmd /c，让 Windows 的命令行解释器来执行：

bash


claude mcp add --scope project image-viewer -- cmd /c npx -y inchat-image-viewer-mcp
参数解释：

参数	作用
--scope project	配置写入当前项目，而不是全局
cmd /c	用 Windows 命令行解释器执行后面的命令
npx -y	自动下载并运行工具，-y 跳过确认提示
成功了
运行完命令后，重新启动 Claude Code，输入 /mcp：

MCP 连接成功
绿色的 ✓ Connected！ 终于成功了。

现在让 Claude Code 查看图片：

成功查看图片
它能看到图片内容了，可以帮我检查截图里有没有隐私信息、描述图片内容、甚至分析 UI 设计。

踩坑总结
整个过程踩了两个坑，都是 Windows 环境下的经典问题：

坑	原因	解决
项目里看不到 MCP	在全局目录安装，项目读不到	加 --scope project，在项目目录下安装
npx 连接失败	Windows 下 npx 是 .cmd 脚本，后台直接调用会崩溃	前面加 cmd /c 套一层
一个通用经验
这次踩坑让我学到了一个非常实用的 Windows 开发经验：

以后不管在什么 AI 工具里配置插件，只要看到 npx 或 npm 的命令，一旦在 Windows 下报错连不上，第一反应就是在最前面加 cmd /c。

这是因为 Windows 的 .cmd 脚本不能被其他程序直接当作子进程调用，必须通过 cmd.exe 来中转。/c 的意思是「执行完就关闭」，不会留下多余的进程。

这个技巧不仅适用于 Claude Code，Cursor、Windsurf 等其他 AI 编程工具遇到类似问题时，大概率也是同一个原因。

写在最后
整个排查过程其实就花了几分钟，但如果没有 Perplexity 帮我定位问题，我可能会在「为什么连不上」这个问题上卡很久。

AI 工具之间的配合越来越有意思了：用 Perplexity 搜索解决方案，给 Claude Code 装上 MCP 插件，然后 Claude Code 就能帮我看图、写博客。每个工具做自己擅长的事，组合起来效率翻倍。
