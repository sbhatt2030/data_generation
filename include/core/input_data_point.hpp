#pragma once

struct InputDataPoint {
    double dev_x = 0.0, dev_y = 0.0, dev_z = 0.0;
    double vff_x = 0.0, vff_y = 0.0, vff_z = 0.0;
    int line_number = -1;
    int vff_mode = 0;  // 0 = collection (apply sign(v_cmd) on kernel side), 1 = testing (inject as-is)
};