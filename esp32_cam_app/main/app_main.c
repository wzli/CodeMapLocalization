void app_camera_main();
void app_httpd_main();
void app_wifi_main();

void app_main() {
    app_wifi_main();
    app_camera_main();
    app_httpd_main();
}
