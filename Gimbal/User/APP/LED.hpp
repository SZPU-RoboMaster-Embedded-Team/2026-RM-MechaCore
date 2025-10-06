#include "../Task/EvenTask.hpp"
#include "../BSP/stdxxx.hpp"

#define RGB_FLOW_COLOR_CHANGE_TIME 500
#define RGB_FLOW_COLOR_LENGHT 3

#define RED 0xFFFF0000
#define GREEN 0xFF00FF00
#define BULE 0xFF0000FF
#define WHITE 0xFFFFFFFF

#define YELLOW 0xFFFFFF00
#define CYAN 0xFF00FFFF
#define PURPLE 0xFFFF00FF
#define BLACK 0xFF000000
#define ORANGE 0xFFFFA500
#define PINK 0xFFFFC0CB

class LED : public IObserver
{
private:
    uint16_t i, j;
    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha, red, green, blue;
    uint32_t aRGB;

    void aRGB_led_show(uint32_t aRGB);

public:
    uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFF0000FF};

    LED(ISubject *sub) : IObserver(sub)
    {
    }

    bool Update();

    void ColorChange(uint16_t inxex);
    void Normal_State();
};
