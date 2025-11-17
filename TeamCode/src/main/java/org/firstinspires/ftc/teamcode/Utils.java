package org.firstinspires.ftc.teamcode;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class Utils {

    public static <T> void applyAction(T[] objs, Consumer<T> action) {
        for (T obj : objs) {
            action.accept(obj);
        }
    }

    public static <T> void applyActions(T[] objs, BiConsumer<T, Integer> action) {
        for (int i = 0; i < objs.length; i++) {
            action.accept(objs[i], i);
        }
    }
}
