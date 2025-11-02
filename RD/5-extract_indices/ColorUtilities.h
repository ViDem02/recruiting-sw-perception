//
// Created by Vi De Matteis on 02/11/25.
//

#ifndef SECOND_PRJ_SETUP_COLORUTILITIES_H
#define SECOND_PRJ_SETUP_COLORUTILITIES_H


class ColorUtilities
{
public:
    struct color {
        int red;
        int green;
        int blue;
    };

    color colors [2] = {
        {255, 0, 0},
        {0, 255, 0}
    };

    [[nodiscard]] color getColor(const int index) const
    {
        return colors[safe_index(index)];
    }

private:

    static int safe_index(const int n) {
        return (n - 1) % sizeof(colors) + 1;
    }

};

#endif //SECOND_PRJ_SETUP_COLORUTILITIES_H