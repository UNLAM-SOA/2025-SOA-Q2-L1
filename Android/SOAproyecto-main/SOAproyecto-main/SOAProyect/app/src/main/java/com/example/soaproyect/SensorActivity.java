package com.example.soaproyect;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Button;
import android.widget.TextView;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

public class SensorActivity extends Activity implements SensorEventListener {


    private static final String MQTT_BROKER = "tcp://broker.hivemq.com:1883";
    private static final String MQTT_CLIENT_ID = "Android_Cuna_Shake";
    private static final String MQTT_TOPIC_CONTROL = "control/estado";
    private static final String PAYLOAD_MOV = "MOVIMIENTO";

    private static final Boolean enabled = true;
    private MqttClient mqttClient;


    private SensorManager sensorManager;
    private Sensor gyroscope;

    // Umbral y temporización para la detección de agitación
    private static final float SHAKE_UMBRAL = 200f;
    private static final long SHAKE_DELAY_MS = 1000L; // 1 segundo de espera entre envíos
    private long lastShakeTime = 0;


    private TextView statusText;
    private Button btnMain;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_shake);

        statusText = findViewById(R.id.status_text);
        btnMain = findViewById(R.id.btn_main);

        // Inicializar Sensores
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        }

        if (gyroscope == null) {
            statusText.setText("Error: Giroscopio no disponible.");
            Log.e("SHAKE", "Giroscopio no disponible en el dispositivo.");
        } else {
            statusText.setText("Iniciando conexión MQTT...");
            initMqtt();
        }

        btnMain.setOnClickListener(v -> backToMainActivity());
        btnMain.setEnabled(enabled);
    }

    private void backToMainActivity(){
        Intent intent = new Intent(SensorActivity.this, MainActivity.class);
        startActivity(intent);
    }

    private void initMqtt() {
        try {
            mqttClient = new MqttClient(MQTT_BROKER, MQTT_CLIENT_ID, new MemoryPersistence());
            final MqttConnectOptions connOpts = new MqttConnectOptions();
            connOpts.setCleanSession(true);
            connOpts.setConnectionTimeout(10);
            connOpts.setKeepAliveInterval(60);

            // Conectar en un hilo secundario para no bloquear la UI
            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        mqttClient.connect(connOpts);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                statusText.setText("MQTT Conectado. Agite el teléfono.");
                            }
                        });
                    } catch (Exception e) {
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                statusText.setText("Error de conexión MQTT: " + e.getMessage());
                            }
                        });
                        Log.e("SHAKE", "Error al conectar MQTT", e);
                    }
                }
            }).start();

        } catch (Exception e) {
            Log.e("SHAKE", "Error al crear cliente MQTT", e);
        }
    }

    private void publishMovement(final String payload) {
        if (mqttClient != null && mqttClient.isConnected()) {
            try {
                MqttMessage message = new MqttMessage(payload.getBytes());
                message.setQos(0);
                message.setRetained(false);

                mqttClient.publish(MQTT_TOPIC_CONTROL, message);

                Log.d("SHAKE", "Publicado: " + payload + " a " + MQTT_TOPIC_CONTROL);
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        statusText.setText("¡MOVIMIENTO ENVIADO! Estado: " + payload);
                    }
                });
            } catch (Exception e) {
                Log.e("SHAKE", "Error al publicar mensaje", e);
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Registrar el listener del sensor con una tasa rápida (SENSOR_DELAY_GAME)
        if (gyroscope != null) {
            sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Desregistrar para ahorrar batería
        if (sensorManager != null) {
            sensorManager.unregisterListener(this);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // Desconectar MQTT
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
            }
        } catch (Exception e) {
            Log.e("SHAKE", "Error al desconectar MQTT", e);
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            // Velocidad angular en X, Y, Z
            float x = event.values[0];
            float y = event.values[1];
            float z = event.values[2];

            // Calcular la magnitud de la velocidad angular al cuadrado
            float angularSpeedSquare = x * x + y * y + z * z;

            long currentTime = System.currentTimeMillis();

            // Si la magnitud supera el umbral y ha pasado el tiempo de delay
            if (angularSpeedSquare > SHAKE_UMBRAL && (currentTime - lastShakeTime) > SHAKE_DELAY_MS) {
                lastShakeTime = currentTime;

                // Publicar el mensaje de control para activar el movimiento en el ESP32
                publishMovement(PAYLOAD_MOV);
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // No necesario para este caso
    }
}