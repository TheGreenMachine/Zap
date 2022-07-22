package com.team1816.lib.events;

import java.util.HashMap;

public class EventAggregator implements IEventAggregator {

    private final HashMap<Object, EventBase> _events = new HashMap<>();

    @Override
    public <TEventType extends EventBase> TEventType GetEvent(Class<TEventType> type) {
        if (_events.containsKey(type)) return type.cast(_events.get(type));
        TEventType newEvent;
        try {
            newEvent = type.getDeclaredConstructor().newInstance();
        } catch (Exception exp) {
            System.out.println(exp.getMessage());
            return null;
        }
        _events.put(type, newEvent);
        return newEvent;
    }
}
