package com.team1816.lib.hardware.factory;

import com.team1816.lib.hardware.RobotConfiguration;
import java.io.InputStream;
import java.util.*;
import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.representer.Representer;

// Since the Collections of configurations are injected by SnakeYaml,
// IDEs will report that the collections are never updated.
@SuppressWarnings("MismatchedQueryAndUpdateOfCollection")
public class YamlConfig {

    private static final Yaml FORMATTER = new Yaml();

    static {
        FORMATTER.setBeanAccess(BeanAccess.FIELD);
    }

    public static RobotConfiguration loadFrom(InputStream input) {
        return loadInternal(input);
    }

    static RobotConfiguration loadInternal(InputStream input) {
        return loadRaw(input);
    }

    static RobotConfiguration loadRaw(InputStream input) {
        Representer representer = new Representer();
        representer.getPropertyUtils().setSkipMissingProperties(true);
        Yaml yaml = new Yaml(new Constructor(RobotConfiguration.class), representer);
        yaml.setBeanAccess(BeanAccess.FIELD);

        return yaml.load(input);
    }

    @Override
    public String toString() {
        return FORMATTER.dump(this);
    }
}
