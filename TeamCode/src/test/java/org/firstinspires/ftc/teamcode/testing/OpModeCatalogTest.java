package org.firstinspires.ftc.teamcode.testing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.BIOBUZZAuto;
import org.firstinspires.ftc.teamcode.MotorTest;
import org.firstinspires.ftc.teamcode.NewTeleop;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JRadioButtonMenuItem;

/** The picker's list: what the Driver Station would show, and what the harness can run. */
public class OpModeCatalogTest {

    private final List<OpModeCatalog.Entry> catalog = OpModeCatalog.discover();

    private OpModeCatalog.Entry find(Class<?> type) {
        for (OpModeCatalog.Entry entry : catalog) {
            if (entry.type == type) return entry;
        }
        return null;
    }

    @Test
    public void itListsTeleOpsAsManualAndAutonomousAsAuto() {
        assertEquals(OpModeCatalog.Kind.TELEOP, find(NewTeleop.class).kind);
        assertEquals(OpModeCatalog.Kind.AUTONOMOUS, find(BIOBUZZAuto.class).kind);
    }

    @Test
    public void itUsesTheDriverStationName() {
        assertEquals(OpModeRunner.displayName(BIOBUZZAuto.class), find(BIOBUZZAuto.class).name);
    }

    @Test
    public void aLinearOpModeIsListedButNotRunnable() {
        OpModeCatalog.Entry motorTest = find(MotorTest.class);
        assertNotNull(motorTest);
        assertFalse(motorTest.runnable());
        assertTrue(motorTest.unsupported.contains("LinearOpMode"));
    }

    @Test
    public void teleOpsComeBeforeAutonomous() {
        boolean seenAuto = false;
        for (OpModeCatalog.Entry entry : catalog) {
            if (entry.kind == OpModeCatalog.Kind.AUTONOMOUS) seenAuto = true;
            else assertFalse("TeleOp " + entry + " after an autonomous one", seenAuto);
        }
    }

    @Test
    public void theMenuTicksTheLoadedOneGreysOutWhatCannotRunAndReportsThePick() {
        final List<OpModeCatalog.Entry> picked = new ArrayList<OpModeCatalog.Entry>();
        OpModeCatalog.Entry loaded = find(BIOBUZZAuto.class);
        JPopupMenu menu = RobotView.buildOpModeMenu(catalog, loaded, picked::add);

        JRadioButtonMenuItem loadedItem = null;
        JRadioButtonMenuItem linearItem = null;
        JRadioButtonMenuItem teleOpItem = null;
        for (int i = 0; i < menu.getComponentCount(); i++) {
            if (!(menu.getComponent(i) instanceof JRadioButtonMenuItem)) continue;
            JRadioButtonMenuItem item = (JRadioButtonMenuItem) menu.getComponent(i);
            if (item.getText().contains("(BIOBUZZAuto)")) loadedItem = item;
            if (item.getText().contains("(MotorTest)")) linearItem = item;
            if (item.getText().contains("(NewTeleop)")) teleOpItem = item;
        }
        assertTrue(loadedItem.isSelected());
        assertFalse(teleOpItem.isSelected());
        assertFalse(linearItem.isEnabled());

        teleOpItem.doClick();
        assertEquals(1, picked.size());
        assertEquals(NewTeleop.class, picked.get(0).type);
    }

    @Test
    public void theMenuHasASectionHeaderForEachKind() {
        JPopupMenu menu = RobotView.buildOpModeMenu(catalog, null, entry -> { });
        int headers = 0;
        for (int i = 0; i < menu.getComponentCount(); i++) {
            Object c = menu.getComponent(i);
            if (c instanceof JMenuItem && !(c instanceof JRadioButtonMenuItem)) headers++;
        }
        assertEquals(2, headers);
    }
}
