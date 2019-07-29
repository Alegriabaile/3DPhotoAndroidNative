package i3d.native0701;

import android.graphics.Bitmap;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    static {
        System.loadLibrary("native-lib");
    }

    private ImageView imageView;
    private Bitmap bitmap;
    private TextView textView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        textView = findViewById(R.id.text_view);
        imageView = findViewById(R.id.image_view);
        bitmap = Bitmap.createBitmap(903, 1203, Bitmap.Config.ARGB_8888);
        int szOfFrame = getImage(bitmap);
        imageView.setImageBitmap(bitmap);

        String str = ("frames.size(): ");
        textView.setText(str + szOfFrame);

        imageView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                bitmap = Bitmap.createBitmap(903, 1203, Bitmap.Config.ARGB_8888);
                getImage(bitmap);
                imageView.setImageBitmap(bitmap);
            }
        });
    }

    public static native int getImage(Bitmap bitmap);
}
