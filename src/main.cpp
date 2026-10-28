#include "Application.h"

int main(int argc, char** argv) {
    App app;
    if (!app.Init(argc, argv)) {
        return 1;
    }
    int code = app.Run();   // --self-test のときはテスト結果を返す
    app.UnInit();
    return code;
}
