package com.team1816.lib.subsystems;

public interface PidProvider {
    double getKP();
    double getKI();
    double getKD();
    double getKF();

    default String pidToString() {
        return String.format(
            "kP = %f, kI = %f, kD = %f, kF = %f",
            getKP(),
            getKI(),
            getKD(),
            getKF()
        );
    }
}
