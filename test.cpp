#include <iostream>
#include <string>

#define SPLIT(x) &x, #x

void printPro(int* pointer, std::string name) {
    std::cout << *pointer << std::endl;
    std::cout << name << std::endl;

}

int main() {
    int hello = 5;
    printPro(SPLIT(hello));
    return 0;
}

