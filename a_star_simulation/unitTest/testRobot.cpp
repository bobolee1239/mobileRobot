//  Copyright 2019 Tsung-Han lee
#include "../include/a_star.h"

int main(int argc, char* argv[]) {
    Robot bot;
    const Robot bot1(Node(3, 3));
    Robot mobileBot("Brian", Node(5, 5));

    std::cout << bot << std::endl;
    std::cout << bot1 << std::endl;
    std::cout << mobileBot.walk(3, 3).walk(7, 5) << std::endl;
    std::cout << mobileBot << std::endl;
    std::cout << mobileBot.getName();
    std::cout << bot1.getPosition();

    return 0;
}
