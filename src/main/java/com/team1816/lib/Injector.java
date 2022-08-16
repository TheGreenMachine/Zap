package com.team1816.lib;

import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Module;
import java.util.ArrayList;
import java.util.List;

/**
 * This wraps the GUICE injector for ease of use
 */
public class Injector {

    private static com.google.inject.Injector _injector;

    private static List<Module> _modules = new ArrayList<>();

    /**
     * This method will initial the injector using the items in the
     * lib module and the passed in season module
     *
     * @param module This is the season module to register
     */
    public static void registerModule(Module module) {
        _modules.add(module);
    }

    public static <T> void register(T instance) {
        _modules.add(
            new AbstractModule() {
                @Override
                protected void configure() {
                    bind((Class) instance.getClass()).toInstance(instance);
                }
            }
        );
    }

    public static <T> void register(Class<T> type, Class<? extends T> instance) {
        _modules.add(
            new AbstractModule() {
                @Override
                protected void configure() {
                    bind(type).to(instance);
                }
            }
        );
    }

    public static <T> T get(Class<T> type) {
        // on first retrieval lock in the modules and create injector
        if (_injector == null) {
            _modules.add(new LibModule());
            _injector = Guice.createInjector(_modules);
        }
        return _injector.getInstance(type);
    }
}
