Lambda expressions are a way of defining anonymous functions (functions without a name) directly within the scope of where they are used. They are concise and can capture variables from their enclosing scope, making them powerful for a variety of programming tasks¹.

A lambda expression has the following syntax:

```c
[capture list] (parameter list) -> return type { function body }
```

The capture list specifies which variables from the outer scope are available inside the lambda. It can be empty, or it can use one of the following forms:

- `[&]` : capture all external variables by reference
- `[=]` : capture all external variables by value
- `[a, &b]` : capture `a` by value and `b` by reference
- `[this]` : capture the `this` pointer of the enclosing class

The parameter list and the return type are similar to those of a normal function. The return type can be omitted if the compiler can deduce it from the function body.

The function body contains the statements that define the behavior of the lambda. It can be a single expression or a block of statements.

Here is an example of a lambda expression that takes two integers as parameters and returns their sum:

```c
auto add = [](int x, int y) -> int { return x + y; };
```

This lambda expression can be assigned to a variable of type `auto` or a suitable function pointer type. It can also be invoked directly, like this:

```c
int result = [](int x, int y) -> int { return x + y; } (3, 4); // result is 7
```

Lambda expressions were introduced in C++11 and have been extended in later versions of the language. Some of the features that have been added are:

- Generic lambdas: The ability to use `auto` in the parameter list to create a template lambda that can accept any type of arguments.
- Mutable lambdas: The ability to modify the captured variables by value by using the `mutable` keyword after the parameter list.
- Capturing `*this` by value: The ability to capture the current object by value instead of by reference by using `[*this]` in the capture list.
- Init-capture: The ability to initialize a captured variable with an expression instead of just copying or referencing an existing variable.
- Variadic capture: The ability to capture a variable number of arguments by using `...` in the capture list.

For more information and examples of lambda expressions in C++, you can refer to these sources:

- [Lambda expression in C++ - GeeksforGeeks](^1^)
- [Lambda expressions (since C++11) - cppreference.com](^2^)
- [Lambda Expressions - Learn C++ For Free | CppTutorial](^3^)
- [Generalized Lambda Expressions in C++14 - GeeksforGeeks](^4^)

Source: Conversation with Bing, 3/4/2024
(1) Lambda expression in C++ - GeeksforGeeks. https://www.geeksforgeeks.org/lambda-expression-in-c/.
(2) Lambda expression in C++ - GeeksforGeeks. https://www.geeksforgeeks.org/lambda-expression-in-c/.
(3) Lambda expressions (since C++11) - cppreference.com. https://en.cppreference.com/w/cpp/language/lambda.
(4) Lambda Expressions - Learn C++ For Free | CppTutorial. https://cpptutorial.io/lambda-expressions/.
(5) Generalized Lambda Expressions in C++14 - GeeksforGeeks. https://www.geeksforgeeks.org/generalized-lambda-expressions-c14/.
