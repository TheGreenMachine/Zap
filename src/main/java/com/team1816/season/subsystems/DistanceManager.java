package com.team1816.season.subsystems;

public class DistanceManager {

    // State
    private final Entry[] buckets;

    // Constants
    public DistanceManager() {
        buckets = distance_buckets;
    }

    static class Entry {

        public final double distance;
        public final double spindexerOutput;
        public final double elevatorOutput;
        public final double shooterVelocity;
        public final double hoodRetracted;

        Entry(
            double distance,
            double spindexerOutput,
            double elevatorOutput,
            double shooterVelocity,
            double hoodRetracted
        ) {
            this.distance = distance;
            this.spindexerOutput = spindexerOutput;
            this.elevatorOutput = elevatorOutput;
            this.shooterVelocity = shooterVelocity;
            this.hoodRetracted = hoodRetracted;
        }

        Entry() {
            this(0, 0, 0, 0, 0);
        }
    }

    private final Entry[] distance_buckets = new Entry[] {
        new Entry(133, .85, 1, 10700, 0),
        new Entry(250, 1.7, 1, 10100, 0),
        new Entry(280, 1.6, 1, 10300, 0),
        new Entry(340, 1.5, 1, 10700, 0),
    };

    private double getSpindexerOutput(double distance) {
//        if(distance < 110){
//            return 0;
//        }
        return .38;
    }

    private double getElevatorOutput(double distance) { // TODO lots of hacks here - generate real outputs with hood in future
//        if(distance < 100){
//            return 0.5;
//        }
//        for (Entry bucket : buckets) {
//            if (distance <= bucket.distance) {
//                return bucket.elevatorOutput;
//            }
//        }
        return .5;
    }

    private double getShooterVelocity(double distance) {
        if(distance < 90){
            return Shooter.NEAR_VELOCITY;
        }
        int x = 0;klmpkbf
        \addplot[][domain=97:105]{+-0.05460540718377771* distance ^3+15.890173490479313* distance ^2+-1475.3520825167316* distance ^1+50635.39042284306* distance ^0};
			\addplot[][domain=105:115]{+0.07594789843630197* distance ^3+-25.234117779845786* distance ^2+2842.6985008674037* distance ^1+-100496.37999560167* distance ^0};
			\addplot[][domain=115:125]{+-0.11763353769937684* distance ^3+41.5514776869634* distance ^2+-4837.644977815653* distance ^1+193916.78668724882* distance ^0};
			\addplot[][domain=125:135]{+0.0945862523612054* distance ^3+-38.03094358575493* distance ^2+5110.15768127414* distance ^1+-220574.99077482586* distance ^0};
			\addplot[][domain=135:140]{+0.21245183712487117* distance ^3+-85.76650541503957* distance ^2+11554.458528227566* distance ^1+-510568.52888773003* distance ^0};
			\addplot[][domain=140:145]{+-0.37285463383765616* distance ^3+160.0622123892219* distance ^2+-22861.56196436904* distance ^1+1095512.427433445* distance ^0};
			\addplot[][domain=145:155]{+0.07621934106436455* distance ^3+-35.2849666931571* distance ^2+5463.779002575915* distance ^1+-273545.7193022279* distance ^0};
			\addplot[][domain=155:165]{+0.06981396070988204* distance ^3+-32.30646482832273* distance ^2+5002.111213526588* distance ^1+-249692.88353467933* distance ^0};
			\addplot[][domain=165:175]{+-0.18047518390389267* distance ^3+91.58666175549574* distance ^2+-15440.254672803461* distance ^1+874637.2402134733* distance ^0};
			\addplot[][domain=175:190]{+0.14898989538137067* distance ^3+-81.3825048692675* distance ^2+14829.349486530107* distance ^1+-891089.6690809848* distance ^0};
			\addplot[][domain=190:200]{+-0.11020660992035931* distance ^3+66.35950315271859* distance ^2+-13241.632037647249* distance ^1+886739.1607835811* distance ^0};
			\addplot[][domain=200:210]{+-0.06231405002095808* distance ^3+37.62396721307784* distance ^2+-7494.5248497191005* distance ^1+503598.68158837117* distance ^0};
			\addplot[][domain=210:220]{+0.05446281000419161* distance ^3+-35.94545460276647* distance ^2+7955.053731608204* distance ^1+-577871.81910454* distance ^0};
        // cubic spline for shooter speed developed through repeated testing
        if(distance < 97) {
            return -0.05460540718377771* distance ^3+15.890173490479313* distance ^2+-1475.3520825167316* distance ^1+50635.39042284306* distance ^0;
        } else if(distance < 105) {
            return -0.017093 * Math.pow(distance, 3) + 4.6524 * Math.pow(distance, 2) - 375.15 * distance + 20465;
        } else if(distance < 115) {
            return 0.025874 * Math.pow(distance, 3) - 9.1400 * Math.pow(distance, 2) + 1100.6 * distance - 32171;
        } else if(distance < 125) {
            return -0.0040377 * Math.pow(distance, 3) + 1.8075 * Math.pow(distance, 2) - 234.96 * distance + 22143;
        } else if(distance < 135){
            return 0.11195 * Math.pow(distance, 3) - 44.822 * Math.pow(distance, 2) + 6013.3 * distance - 256950;
        } else if(distance < 140){
            return -0.19873 * Math.pow(distance, 3) + 89.442 * Math.pow(distance, 2) - 13328.0 * distance + 671810;
        } else if(distance < 145){
            return 0.025159 * Math.pow(distance, 3) - 13.993 * Math.pow(distance, 2) + 2601.0 * distance - 145880;
        } else if(distance < 155){
            return 0.039889 * Math.pow(distance, 3) - 21.240 * Math.pow(distance, 2) + 3789.5 * distance - 210850;
        } else if(distance < 165){
            return 0.025159 * Math.pow(distance, 3) - 13.993 * Math.pow(distance, 2) + 2601.0 * distance - 145880;
        } else if(distance < 175){
            return 0.039889 * Math.pow(distance, 3) - 21.240 * Math.pow(distance, 2) + 3789.5 * distance - 210850;
        } else if(distance < 190){
            return 0.025159 * Math.pow(distance, 3) - 13.993 * Math.pow(distance, 2) + 2601.0 * distance - 145880;
        } else if(distance < 200){
            return 0.039889 * Math.pow(distance, 3) - 21.240 * Math.pow(distance, 2) + 3789.5 * distance - 210850;
        } else if(distance < 210){
            return 0.025159 * Math.pow(distance, 3) - 13.993 * Math.pow(distance, 2) + 2601.0 * distance - 145880;
        } else {
            return -0.012978 * Math.pow(distance, 3) + 7.9424 * Math.pow(distance, 2) - 1580.0 * distance + 118480;
        }
//        return (distance * 51.8) + 6600; // linear equation generated by testing @ MURA - 52-5500
    }

    private double getHoodRetracted(double distance) {
        if(distance < 90){
            return 0;
        }
        return 1;
    }

    public double getOutput(double distance, SUBSYSTEM subsystem) {
        switch (subsystem) {
            case SPINDEXER:
                return getSpindexerOutput(distance);
            case ELEVATOR:
                return getElevatorOutput(distance);
            case SHOOTER:
                return getShooterVelocity(distance);
            case HOOD:
                return getHoodRetracted(distance);
        }
        System.out.println("not a SUBSYSTEM!");
        return 0;
    }

    public enum SUBSYSTEM {
        SPINDEXER,
        ELEVATOR,
        SHOOTER,
        HOOD,
    }
}
