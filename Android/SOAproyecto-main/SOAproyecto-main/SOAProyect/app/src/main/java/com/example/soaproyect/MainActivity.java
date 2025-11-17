package com.example.soaproyect;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "MQTT_CLIENT_APP";
    private MqttClient mqttClient;
    private static final String MQTT_SERVER = "tcp://broker.hivemq.com:1883";
    private static final String TOPIC_CONTROL = "control/estado";
    private static final String TOPIC_SONIDO = "sensor/sonido";
    private static final String TOPIC_MOV = "sensor/movimiento";

    private static final int RECONNECT_DELAY_MS = 5000;

    private TextView textSonido, textMovimiento, textStatus;
    private Button buttonOn, buttonOff, btnShakeAccess;

    private final Handler handler = new Handler(Looper.getMainLooper());

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        buttonOn = findViewById(R.id.buttonOn);
        buttonOff = findViewById(R.id.buttonOff);
        textSonido = findViewById(R.id.textSonido);
        textMovimiento = findViewById(R.id.textMovimiento);
        textStatus = findViewById(R.id.textStatus);
        btnShakeAccess = findViewById(R.id.btn_shake_access);



        setControlsEnabled(false);


        new Thread(this::connectAndSubscribeMqtt).start();


        buttonOn.setOnClickListener(v -> publishMessage("ACTIVO"));
        buttonOff.setOnClickListener(v -> publishMessage("STANDBY"));
        btnShakeAccess.setOnClickListener(v -> openShakeActivity());
    }

    private void openShakeActivity() {
        Intent intent = new Intent(MainActivity.this, SensorActivity.class);
        startActivity(intent);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();

        handler.removeCallbacksAndMessages(null);

        if (mqttClient != null) {
            try {
                if (mqttClient.isConnected()) {
                    mqttClient.disconnect();
                }
                mqttClient.close();
            } catch (MqttException e) {
                Log.e(TAG, "Error al desconectar/cerrar el cliente MQTT.", e);
            }
        }
    }

    private void connectAndSubscribeMqtt() {
        if (mqttClient != null && mqttClient.isConnected()) {
            // Evitar reconexiones si ya está conectado
            return;
        }

        try {
            // Inicializar solo si es null (no necesario en reconexión)
            if (mqttClient == null) {
                mqttClient = new MqttClient(MQTT_SERVER, MqttClient.generateClientId(), null);
                mqttClient.setCallback(mqttCallback);
            }

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);

            // Intentar conectar
            mqttClient.connect(options);


            mqttClient.subscribe(TOPIC_SONIDO);
            mqttClient.subscribe(TOPIC_MOV);

            Log.i(TAG, "Conexión y suscripción MQTT exitosa.");
            // Informar al usuario y habilitar controles
            updateStatus("Conectado", true);

            handler.removeCallbacksAndMessages(null);

        } catch (MqttException e) {
            Log.e(TAG, "Fallo la conexión MQTT: " + e.getMessage(), e);
            // Informar al usuario del fallo y programar reconexión
            updateStatus("Desconectado: " + e.getMessage(), false);

            scheduleReconnect();
        }
    }

    private void scheduleReconnect() {
        handler.postDelayed(() -> {
            Log.d(TAG, "Intentando reconectar después de " + RECONNECT_DELAY_MS + "ms...");
            new Thread(this::connectAndSubscribeMqtt).start();
        }, RECONNECT_DELAY_MS);
    }

    private final MqttCallback mqttCallback = new MqttCallback() {
        @Override
        public void connectionLost(Throwable cause) {
            Log.e(TAG, "Conexión MQTT perdida.", cause);
            updateStatus("Conexión Perdida. Intentando reconectar...", false);

            scheduleReconnect();
        }

        @Override
        public void messageArrived(String topic, MqttMessage message) {
            runOnUiThread(() -> {
                String payload = new String(message.getPayload());

                if (topic.equals(TOPIC_SONIDO)) {
                    textSonido.setText("Sonido: " + payload);
                } else if (topic.equals(TOPIC_MOV)) {
                    textMovimiento.setText("Movimiento: " + payload);
                }
                Log.d(TAG, "Mensaje recibido - Tópico: " + topic + ", Mensaje: " + payload);
            });
        }

        @Override
        public void deliveryComplete(IMqttDeliveryToken token) {}
    };

    private void publishMessage(String msg) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            Toast.makeText(this, "Error: No conectado. Intente de nuevo.", Toast.LENGTH_SHORT).show();
            return;
        }

        try {
            MqttMessage message = new MqttMessage(msg.getBytes());
            message.setQos(0);
            mqttClient.publish(TOPIC_CONTROL, message);

            Log.i(TAG, "Comando enviado: " + msg);
            Toast.makeText(this, "Comando enviado: " + msg, Toast.LENGTH_SHORT).show();

        } catch (MqttException e) {
            Log.e(TAG, "Error al publicar el mensaje: " + msg, e);
            Toast.makeText(this, "Error al enviar el comando. Revise la conexión.", Toast.LENGTH_LONG).show();
        }
    }

    private void updateStatus(String status, boolean isConnected) {
        runOnUiThread(() -> {
            if (textStatus != null) {
                textStatus.setText("Estado: " + status);
            }

            setControlsEnabled(isConnected);
        });
    }

    private void setControlsEnabled(boolean enabled) {
        runOnUiThread(() -> {
            buttonOn.setEnabled(enabled);
            buttonOff.setEnabled(enabled);

            if (btnShakeAccess != null) {
                btnShakeAccess.setEnabled(enabled);
            }
        });
    }
}