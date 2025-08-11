#include "/home/legaca/async-tcp/async-sockets/include/tcpserver.hpp"
#include "/home/legaca/async-tcp/async-sockets/include/tcpsocket.hpp"
#include <iostream>
#include <unordered_map>
#include <string>

using namespace std;

const char* serverAddress = "sincap.local";
int serverPort = 12345;
int sock;
string seritData;
string engelData = "Engel yok";
string oncekiSeritData = "#100*";

bool espbaglanti = false;
bool hizgonderildi = false;

unordered_map<TCPSocket*, string> clientNames;

int connectToServer(const char* serverAddress, int serverPort) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        cerr << "Socket oluşturulamadı!" << endl;
        return -1;
    }

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    int status = getaddrinfo(serverAddress, to_string(serverPort).c_str(), &hints, &res);
    if (status != 0) {
        cerr << "getaddrinfo hatası: " << gai_strerror(status) << endl;
        close(sock);
        return -1;
    }

    if (connect(sock, res->ai_addr, res->ai_addrlen) < 0) {
        cerr << "Bağlantı hatası!" << endl;
        close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    return sock;
}

bool sendDataToServer(int sock, const string& message) {
    if (send(sock, message.c_str(), message.length(), 0) < 0) {
        cerr << "Veri gönderilemedi!" << endl;
        return false;
    }
    cout << "Veri gönderildi: " << message << endl;
    return true;
}

void server() {
    TCPServer tcpServer;

    tcpServer.onNewConnection = [&](TCPSocket *newClient) {
        cout << "Yeni istemci bağlandı: [" << newClient->remoteAddress() << ":" << newClient->remotePort() << "]" << endl;

        newClient->onMessageReceived = [newClient](string message) {
            size_t start = message.find('#');
            size_t end = message.find('*');
            if (start != string::npos && end != string::npos && start < end) {
                string value = message.substr(start + 1, end - start - 1);

                // İstemci adı belirleme
                if (clientNames.find(newClient) == clientNames.end()) {
                    clientNames[newClient] = value;
                    cout << "İstemci adı belirlendi: " << value << endl;
                } else {
                    // İstemci adı zaten belirlenmişse, gelen veriyi işle
                    string clientName = clientNames[newClient];
                    if (clientName == "engel") {
                        // Engelden gelen veri
                        string engelData = value;
                        cout << "Engelden gelen veri => " << engelData << endl;
                    } else if (clientName == "serit") {
                        // Şeritten gelen veri
                        seritData = value;
                        cout << "Şeritten gelen veri => " << seritData << endl;
                    } else {
                            cout << "Veri doğru formatta değil!" << endl;
                        }

                        if (espbaglanti)
                        {
                        cout << "ESP bağlantı 1" << endl;
                        if (engelData == "Engel yok")
                        { // engel yok yada 4 m
                            if (hizgonderildi == false)
                                {
                                    hizgonderildi = true;
                                sendDataToServer(sock, "SN590\n");
                                
                                }
                            if (oncekiSeritData != seritData) {
                            if (sendDataToServer(sock, seritData))
                             {
                                cout << "gonderildi ===>" << seritData << endl;
                             }
                            }else {
                            cout << "HATA" << endl;
                            }
                            oncekiSeritData = seritData;
                            }
                        }
                    }
                }
            else {
                cout << "Geçersiz format: " << message << endl;
                newClient->Send("Geçersiz format! Mesaj # ve * ile başlamalı ve bitmelidir.");
            }
        };

        newClient->onSocketClosed = [newClient](int errorCode) {
            cout << "Soket kapatıldı: " << newClient->remoteAddress() << ":" << newClient->remotePort() << " -> " << errorCode << endl;
            clientNames.erase(newClient);
        };
    };

    tcpServer.Bind(8888, [](int errorCode, string errorMessage) {
        cout << errorCode << " : " << errorMessage << endl;
    });

    tcpServer.Listen([](int errorCode, string errorMessage) {
        cout << errorCode << " : " << errorMessage << endl;
    });

    string input;
    while (getline(cin, input)) {
        // Server çalışırken kullanıcıdan giriş alabilir
    }

    tcpServer.Close();
}

int main() {
    sock = connectToServer(serverAddress, serverPort);
    if (sock < 0) {
        espbaglanti = false;
        cout << "Hata! | ESP'ye bağlanılamadı!" << endl;
    } else {
        espbaglanti = true;
        cout << "Başarılı! | ESP'ye bağlanıldı!" << endl;
    }
    server();
}

// Compilation command:
// g++ -o dav dav.cpp -I/home/legaca/async-tcp/async-sockets/include -lpthread