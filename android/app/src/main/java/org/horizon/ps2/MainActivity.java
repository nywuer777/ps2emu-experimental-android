package org.horizon.ps2;

import android.app.Activity;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.view.Gravity;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class MainActivity extends Activity {
    static {
        System.loadLibrary("ps2emu_core");
        System.loadLibrary("ps2emu_jni");
    }

    private native long nativeCreate();
    private native void nativeDestroy(long handle);
    private native int nativeLoadBios(long handle, String path);
    private native int nativeLoadDisc(long handle, String path);
    private native int nativeLoadDiscElf(long handle, String path);
    private native int nativeRun(long handle, long steps);
    private native String nativeStatus(long handle);

    private final ExecutorService worker = Executors.newSingleThreadExecutor();
    private long core;
    private String biosPath;
    private String isoPath;
    private TextView status;
    private Button biosButton;
    private Button discButton;
    private Button runButton;
    private Button directRunButton;
    private Button continueButton;
    private boolean machineStarted;

    private static final int PICK_BIOS = 1;
    private static final int PICK_ISO = 2;
    private static final long PROBE_STEPS = 16_000_000L;
    private static final long DIRECT_PROBE_STEPS = 1_000_000L;
    private static final long PROGRESS_INTERVAL = 8L * 1024L * 1024L;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        core = nativeCreate();

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(28, 48, 28, 24);
        root.setGravity(Gravity.CENTER_VERTICAL);
        root.setBackgroundColor(0xff101114);
        root.setOnApplyWindowInsetsListener((view, insets) -> {
            view.setPadding(28, 48 + insets.getSystemWindowInsetTop(), 28,
                    24 + insets.getSystemWindowInsetBottom());
            return insets;
        });

        TextView title = new TextView(this);
        title.setText("PS2 Experimental");
        title.setTextSize(28);
        title.setTextColor(0xffffffff);
        root.addView(title);

        TextView note = new TextView(this);
        note.setText("Diagnóstico ARM64 do núcleo experimental. O boot de jogos ainda não está validado.");
        note.setTextSize(16);
        note.setTextColor(0xffc6c6cc);
        root.addView(note);

        biosButton = button("Selecionar BIOS");
        discButton = button("Selecionar ISO");
        runButton = button("Executar BIOS até exceção (limite 16M)");
        directRunButton = button("Executar ELF direto até exceção (limite 1M)");
        continueButton = button("Continuar EE até próxima exceção (limite 1M)");
        status = new TextView(this);
        status.setTextColor(0xffffffff);
        status.setTextSize(14);
        status.setTextIsSelectable(true);

        ScrollView statusScroller = new ScrollView(this);
        statusScroller.setFillViewport(true);
        statusScroller.setVerticalScrollBarEnabled(true);
        statusScroller.addView(status);

        root.addView(biosButton);
        root.addView(discButton);
        root.addView(runButton);
        root.addView(directRunButton);
        root.addView(continueButton);
        root.addView(statusScroller, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f));
        setContentView(root);

        biosButton.setOnClickListener(view -> pick(PICK_BIOS));
        discButton.setOnClickListener(view -> pick(PICK_ISO));
        runButton.setOnClickListener(view -> run());
        directRunButton.setOnClickListener(view -> runDirectElf());
        continueButton.setOnClickListener(view -> continueExecution());
        refresh();
    }

    private Button button(String label) {
        Button button = new Button(this);
        button.setText(label);
        return button;
    }

    private void pick(int requestCode) {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.setType("*/*");
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        startActivityForResult(intent, requestCode);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode != RESULT_OK || data == null || data.getData() == null) return;
        if (requestCode != PICK_BIOS && requestCode != PICK_ISO) return;

        final boolean isBios = requestCode == PICK_BIOS;
        final Uri uri = data.getData();
        setControlsEnabled(false);
        status.setText(isBios
                ? "Importando e validando a BIOS..."
                : "Copiando a ISO para o armazenamento do app. A tela permanece ativa durante a cópia...");

        worker.execute(() -> {
            String fileName = isBios ? "bios.bin" : "disc.iso";
            try {
                String path = copy(uri, fileName, isBios);
                int result = isBios
                        ? nativeLoadBios(core, path)
                        : nativeLoadDisc(core, path);
                String nativeMessage = nativeStatus(core);
                runOnUiThread(() -> {
                    if (isFinishing() || isDestroyed()) return;
                    if (result == 0) {
                        if (isBios) biosPath = path;
                        else isoPath = path;
                        machineStarted = false;
                        status.setText((isBios
                                ? "BIOS carregada nos mapas EE e IOP.\n"
                                : "ISO montada no CDVD para inicialização pela BIOS.\n")
                                + nativeMessage);
                    } else {
                        status.setText((isBios ? "Falha ao carregar BIOS" : "Falha ao analisar ISO")
                                + " (" + result + ").\n" + nativeMessage);
                    }
                    setControlsEnabled(true);
                });
            } catch (Exception error) {
                runOnUiThread(() -> {
                    if (isFinishing() || isDestroyed()) return;
                    status.setText("Falha ao importar " + (isBios ? "BIOS" : "ISO")
                            + ": " + error.getMessage());
                    setControlsEnabled(true);
                });
            }
        });
    }

    private String copy(Uri uri, String name, boolean isBios) throws IOException {
        long expectedBytes = querySize(uri);
        File destination = new File(getFilesDir(), name);
        long copiedBytes = 0;
        long nextReportAt = PROGRESS_INTERVAL;

        try (InputStream input = getContentResolver().openInputStream(uri);
             FileOutputStream output = new FileOutputStream(destination, false)) {
            if (input == null) throw new IOException("o provedor não abriu o arquivo");
            byte[] buffer = new byte[256 * 1024];
            int count;
            while ((count = input.read(buffer)) != -1) {
                output.write(buffer, 0, count);
                copiedBytes += count;
                if (!isBios && copiedBytes >= nextReportAt) {
                    nextReportAt = copiedBytes + PROGRESS_INTERVAL;
                    final long progress = copiedBytes;
                    runOnUiThread(() -> {
                        if (isFinishing() || isDestroyed()) return;
                        status.setText(formatCopyProgress(progress, expectedBytes));
                    });
                }
            }
            output.getFD().sync();
        }
        return destination.getAbsolutePath();
    }

    private long querySize(Uri uri) {
        try (Cursor cursor = getContentResolver().query(uri,
                new String[]{OpenableColumns.SIZE}, null, null, null)) {
            if (cursor != null && cursor.moveToFirst()) {
                int column = cursor.getColumnIndex(OpenableColumns.SIZE);
                if (column >= 0 && !cursor.isNull(column)) return cursor.getLong(column);
            }
        } catch (Exception ignored) {
            // Some document providers do not report a file size.
        }
        return -1;
    }

    private String formatCopyProgress(long copied, long total) {
        double copiedMb = copied / (1024.0 * 1024.0);
        if (total > 0) {
            double totalMb = total / (1024.0 * 1024.0);
            return String.format(Locale.ROOT, "Copiando ISO: %.0f de %.0f MB", copiedMb, totalMb);
        }
        return String.format(Locale.ROOT, "Copiando ISO: %.0f MB", copiedMb);
    }

    private void setControlsEnabled(boolean enabled) {
        biosButton.setEnabled(enabled);
        discButton.setEnabled(enabled);
        runButton.setEnabled(enabled);
        directRunButton.setEnabled(enabled);
        continueButton.setEnabled(enabled);
    }

    private void run() {
        if (biosPath == null || isoPath == null) {
            status.setText("Selecione uma BIOS e uma ISO válidas primeiro.");
            return;
        }
        setControlsEnabled(false);
        machineStarted = true;
        status.setText("Executando o probe do núcleo...");
        worker.execute(() -> {
            int result = nativeRun(core, PROBE_STEPS);
            String nativeMessage = nativeStatus(core);
            runOnUiThread(() -> {
                if (isFinishing() || isDestroyed()) return;
                status.setText((result == 0
                        ? "Probe concluído.\n"
                        : "Probe interrompido (" + result + ").\n") + nativeMessage);
                setControlsEnabled(true);
            });
        });
    }

    private void runDirectElf() {
        if (isoPath == null) {
            status.setText("Selecione uma ISO com SYSTEM.CNF e ELF de boot.");
            return;
        }
        setControlsEnabled(false);
        machineStarted = false;
        status.setText("Carregando o ELF da ISO e executando sem BIOS...");
        worker.execute(() -> {
            int loadResult = nativeLoadDiscElf(core, isoPath);
            int runResult = loadResult == 0
                    ? nativeRun(core, DIRECT_PROBE_STEPS)
                    : loadResult;
            machineStarted = loadResult == 0;
            String nativeMessage = nativeStatus(core);
            runOnUiThread(() -> {
                if (isFinishing() || isDestroyed()) return;
                status.setText((runResult == 0
                        ? "Probe ELF direto concluído. Este caminho não representa boot completo.\n"
                        : "Probe ELF direto interrompido (" + runResult + ").\n")
                        + nativeMessage);
                setControlsEnabled(true);
            });
        });
    }

    private void continueExecution() {
        if (!machineStarted) {
            status.setText("Execute primeiro o probe da BIOS ou o ELF direto.");
            return;
        }
        setControlsEnabled(false);
        status.setText("Continuando do PC atual até a próxima exceção do EE...");
        worker.execute(() -> {
            int result = nativeRun(core, DIRECT_PROBE_STEPS);
            String nativeMessage = nativeStatus(core);
            runOnUiThread(() -> {
                if (isFinishing() || isDestroyed()) return;
                status.setText((result == 0
                        ? "Continuação concluída; estado preservado entre probes.\n"
                        : "Continuação interrompida (" + result + ").\n") + nativeMessage);
                setControlsEnabled(true);
            });
        });
    }

    private void refresh() {
        status.setText(core == 0
                ? nativeStatus(core)
                : "Selecione BIOS e ISO. Esta build valida a leitura e o probe, não inicia uma partida.");
    }

    @Override
    protected void onDestroy() {
        worker.execute(() -> {
            if (core != 0) {
                nativeDestroy(core);
                core = 0;
            }
        });
        worker.shutdown();
        super.onDestroy();
    }
}
