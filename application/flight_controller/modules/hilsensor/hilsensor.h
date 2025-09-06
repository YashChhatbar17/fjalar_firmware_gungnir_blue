typedef struct hil_data {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float p;

    float lon;
    float lat;
    float alt;

    uint32_t time;

    bool HIL_awaiting_init;
    bool HIL_awaiting_launch;
} hil_data_t;

int hilsensor_feed(const struct device *dev, hil_data_t *hil);