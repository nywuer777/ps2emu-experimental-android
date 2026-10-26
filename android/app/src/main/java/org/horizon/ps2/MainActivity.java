package org.horizon.ps2;

import android.app.Activity;
import android.os.Bundle;
import android.content.Intent;
import android.net.Uri;
import android.view.Gravity;
import android.widget.*;
import java.io.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public final class MainActivity extends Activity {
    static { System.loadLibrary("ps2emu_core"); System.loadLibrary("ps2emu_jni"); }
    private native long nativeCreate(); private native void nativeDestroy(long h);
    private native int nativeLoadBios(long h,String p); private native int nativeLoadDisc(long h,String p);
    private native int nativeRun(long h,long steps); private native String nativeStatus(long h);
    private long core; private String bios,iso; private final ExecutorService worker=Executors.newSingleThreadExecutor();
    private TextView status; private Button biosButton,discButton,runButton;
    private static final int PICK_BIOS=1,PICK_ISO=2;
    @Override public void onCreate(Bundle b){super.onCreate(b);core=nativeCreate();
      LinearLayout root=new LinearLayout(this);root.setOrientation(1);root.setPadding(28,48,28,24);root.setGravity(Gravity.CENTER_VERTICAL);root.setBackgroundColor(0xff101114);
      TextView title=new TextView(this);title.setText("PS2 Experimental");title.setTextSize(28);title.setTextColor(-1);root.addView(title);
      TextView note=new TextView(this);note.setText("Diagnóstico ARM64 do núcleo experimental. O boot de jogos ainda não está validado.");note.setTextSize(16);note.setTextColor(0xffc6c6cc);root.addView(note);
      biosButton=button("Selecionar BIOS");discButton=button("Selecionar ISO");runButton=button("Executar 100 mil passos");status=new TextView(this);status.setTextColor(-1);status.setTextSize(14);
      root.addView(biosButton);root.addView(discButton);root.addView(runButton);root.addView(status);setContentView(root);
      biosButton.setOnClickListener(v->pick(PICK_BIOS));discButton.setOnClickListener(v->pick(PICK_ISO));runButton.setOnClickListener(v->run());refresh(); }
    private Button button(String s){Button b=new Button(this);b.setText(s);return b;}
    private void pick(int code){Intent i=new Intent(Intent.ACTION_OPEN_DOCUMENT);i.setType("*/*");i.addCategory(Intent.CATEGORY_OPENABLE);startActivityForResult(i,code);}
    @Override protected void onActivityResult(int req,int res,Intent data){super.onActivityResult(req,res,data);if(res!=RESULT_OK||data==null)return;try{String p=copy(data.getData(),req==PICK_BIOS?"bios.bin":"disc.iso");if(req==PICK_BIOS){bios=p;int r=nativeLoadBios(core,bios);status.setText(r==0?"BIOS carregada nos mapas EE e IOP.\n"+nativeStatus(core):"Falha ao ler BIOS ("+r+")");}else{iso=p;int r=nativeLoadDisc(core,iso);status.setText(r==0?"ELF de SYSTEM.CNF carregado diretamente, sem boot da BIOS.\n"+nativeStatus(core):"Falha ao abrir ISO/ELF ("+r+")");}}catch(Exception e){status.setText("Falha ao importar arquivo: "+e.getMessage());}}
    private String copy(Uri uri,String name)throws Exception{File f=new File(getFilesDir(),name);try(InputStream in=getContentResolver().openInputStream(uri);OutputStream out=new FileOutputStream(f)){byte[]buf=new byte[65536];int n;while((n=in.read(buf))>0)out.write(buf,0,n);}return f.getAbsolutePath();}
    private void run(){if(bios==null||iso==null){status.setText("Selecione BIOS e ISO primeiro.");return;}runButton.setEnabled(false);worker.execute(()->{int r=nativeRun(core,100000);runOnUiThread(()->{runButton.setEnabled(true);status.setText((r==0?"Execução concluída.\n":"Execução interrompida ("+r+").\n")+nativeStatus(core));});});}
    private void refresh(){status.setText("Selecione BIOS e ISO. Este app expõe o probe integrado, não garante execução jogável.");}
    @Override protected void onDestroy(){worker.shutdownNow();nativeDestroy(core);super.onDestroy();}
}
