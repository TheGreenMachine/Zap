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
        //        if(distance < 90) {
        //            return Shooter.NEAR_VELOCITY;
        //        } else if(distance < 105) {
        //            return -0.05460540718377771 * Math.pow(distance,3) + 15.890173490479313 * Math.pow(distance, 2) - 1475.3520825167316 * distance + 50635.39042284306;
        //        } else if(distance < 115) {
        //            return 0.07594789843630197 * Math.pow(distance, 3) - 25.234117779845786 * Math.pow(distance, 2) + 2842.6985008674037 * distance - 100496.37999560167;
        //        } else if(distance < 125) {
        //            return -0.11763353769937684* Math.pow(distance, 3) + 41.5514776869634 * Math.pow(distance, 2) - 4837.644977815653 * distance + 193916.78668724882;
        //        } else if(distance < 135){
        //            return 0.0945862523612054* Math.pow(distance, 3) - 38.03094358575493* Math.pow(distance, 2) + 5110.15768127414 * distance - 220574.99077482586;
        //        } else if(distance < 140){
        //            return 0.21245183712487117 * Math.pow(distance, 3) - 85.76650541503957 * Math.pow(distance, 2) + 11554.458528227566* distance -510568.52888773003;
        //        } else if(distance < 145){
        //            return -0.37285463383765616* Math.pow(distance, 3) + 160.0622123892219* Math.pow(distance, 2) - 22861.56196436904* distance + 1095512.427433445;
        //        } else if(distance < 155){
        //            return 0.07621934106436455* Math.pow(distance, 3) -35.2849666931571* Math.pow(distance, 2) + 5463.779002575915 * distance - 273545.7193022279;
        //        } else if(distance < 165){
        //            return 0.06981396070988204* Math.pow(distance, 3) - 32.30646482832273* Math.pow(distance, 2) + 5002.111213526588 * distance - 249692.88353467933;
        //        } else if(distance < 175){
        //            return -0.18047518390389267* Math.pow(distance, 3) + 91.58666175549574* Math.pow(distance, 2) - 15440.254672803461* distance + 874637.2402134733;
        //        } else if(distance < 190){
        //            return 0.14898989538137067* Math.pow(distance, 3) - 81.3825048692675 * Math.pow(distance, 2) + 14829.349486530107 * distance - 891089.6690809848;
        //        } else if(distance < 200){
        //            return -0.11020660992035931* Math.pow(distance, 3) + 66.35950315271859 * Math.pow(distance, 2) - 13241.632037647249* distance + 886739.1607835811;
        //        } else if(distance < 210){
        //            return -0.06231405002095808* Math.pow(distance, 3) + 37.62396721307784* Math.pow(distance, 2) - 7494.5248497191005 * distance + 503598.68158837117;
        //        } else {
        //            return 0.05446281000419161* Math.pow(distance, 3) - 35.94545460276647* Math.pow(distance, 2) + 7955.053731608204* distance - 577871.81910454;
        //        }

        if (distance < 80) {
            return Shooter.NEAR_VELOCITY;
        } else if (distance < 105) {
            return 62.5 * distance + 1137.5;
        } else if (distance < 115) {
            return 50 * distance + 2450;
        } else if (distance < 125) {
            return 50 * distance + 2450;
        } else if (distance < 135) {
            return 20 * distance + 6200;
        } else if (distance < 140) {
            return 20 * distance + 6200;
        } else if (distance < 145) {
            return 40 * distance + 3400;
        } else if (distance < 155) {
            return 25 * distance + 5575.5;
        } else if (distance < 165) {
            return 27.5 * distance + 5187.5;
        } else if (distance < 175) {
            return 47.5 * distance + 2287.5;
        } else if (distance < 190) {
            return 20 * distance + 6700;
        } else if (distance < 200) {
            return 64 * distance - 1660;
        } else if (distance < 210) {
            return 73.5 * distance - 3560;
        } else {
            return 52.5 * distance + 850;
        }
    }

    private double getHoodRetracted(double distance) {
        if (distance < 90) {
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
