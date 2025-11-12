module;
#include <iostream>
export module hello;

export void say_hello() {
    std::cout << "Hello from a lot of modules!\n";
}