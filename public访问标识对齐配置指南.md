# public 访问标识对齐配置指南

## 目标

将 C++ 类中的 `public:`、`protected:` 和 `private:` 与 `class` 关键字对齐，类成员仍保持 4 个空格缩进。

```cpp
class Motor {
public:
    void start();

private:
    int state_;
};
```

## 需要修改的文件

在仓库根目录 `.clang-format` 的 `IndentWidth: 4` 附近增加：

```yaml
IndentAccessModifiers: false
AccessModifierOffset: -4
```

其中：

- `IndentWidth: 4` 表示普通类成员缩进 4 个空格。
- `AccessModifierOffset: -4` 让访问标识相对普通成员向左移动 4 列，因此正好与 `class` 对齐。
- `IndentAccessModifiers: false` 确保 clang-format 使用 `AccessModifierOffset`；如果将它设为 `true`，`AccessModifierOffset` 会被忽略。

## 自检

保存 `.clang-format` 后，在 VS Code 中对一个包含 C++ `class` 的文件执行“Format Document”。检查结果：

- `public:`、`protected:` 和 `private:` 前面的空格数与当前 `class` 关键字一致。
- 类成员比访问标识多 4 个空格。
- 嵌套在命名空间或其他作用域中时，访问标识也会和它所属的 `class` 对齐。

## 本次边界

本任务没有再次禁用“古法编程模式”，因此本次只写入指南，未直接修改 `.clang-format`，也未格式化任何源码。
