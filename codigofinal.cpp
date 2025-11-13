#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <filesystem>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <math.h>
#include <nvml.h>

using namespace std;

bool flag = false;

// --- Funciones Config ---
map<string, string> readConfig(string fichero)
{
    map<string, string> kv;
    ifstream cFile(fichero);
    if (cFile.is_open())
    {
        string line;
        while (getline(cFile, line))
        {
            line.erase(remove_if(line.begin(), line.end(), [](unsigned char x) { return isspace(x); }), line.end());
            if (line.empty() || line[0] == '#')
                continue;
            auto delimiterPos1 = line.find('=');
            auto delimiterPos2 = line.find('#');
            auto delimiterPos12 = (delimiterPos2 != string::npos ? delimiterPos2 : line.size()) - delimiterPos1;
            auto key = line.substr(0, delimiterPos1);
            auto value = line.substr(delimiterPos1 + 1, delimiterPos12 - 1);
            kv.insert({key, value});
        }
        cFile.close();
    }
    else
    {
        cerr << "Couldn't open config file for reading.\n";
    }
    return kv;
}

// --- Funciones GPU ---
vector<int> getGpuIds(const string &gpuStr)
{
    vector<int> gpuIds;
    stringstream ss(gpuStr);
    string token;
    while (getline(ss, token, ','))
        gpuIds.push_back(stoi(token));
    return gpuIds;
}

map<string, long> getGpuEnergy_uj(const nvmlDevice_t &device, int id, const string &name)
{
    map<string, long> energy;
    unsigned long long energia_mJ;
    if (nvmlDeviceGetTotalEnergyConsumption(device, &energia_mJ) == NVML_SUCCESS)
    {
        long energia_uJ = energia_mJ * 1000;
        string key = name + ";" + to_string(id);
        energy.insert({key, energia_uJ});
    }
    return energy;
}

// --- Funciones CPU ---
map<string, long> getCpuEnergy_uj(const string &subdomain)
{
    char delimiter = ',';
    vector<string> subd;
    string sdCopy = subdomain;
    size_t pos = 0;
    string path = "/sys/class/powercap/intel-rapl";
    string cadena = "intel-rapl";
    string Fenergy_uj = "/energy_uj";
    string Fname = "/name";
    string dir, name, ruta, file, energia, pathing;
    ifstream eFile;
    char dom;
    map<string, long> energy;

    while ((pos = sdCopy.find(delimiter)) != string::npos)
    {
        subd.push_back(sdCopy.substr(0, pos));
        sdCopy.erase(0, pos + 1);
    }
    subd.push_back(sdCopy);

    for (auto &sub : subd)
    {
        dom = sub.substr(0, sub.find('-')).c_str()[0];
        name = sub.substr(sub.find('-') + 1);

        for (const auto &entry : filesystem::directory_iterator(path))
        {
            dir = entry.path().filename();
            pathing = entry.path();

            if (dir.find(cadena) != string::npos)
            {
                if (dir.back() == dom)
                {
                    ruta = pathing + Fname;
                    eFile.open(ruta);
                    if (eFile.is_open())
                    {
                        getline(eFile, file);
                        file = file.substr(0, file.find('-'));
                        eFile.close();
                    }

                    if (file == name)
                    {
                        ruta = pathing + Fenergy_uj;
                        eFile.open(ruta);
                        if (eFile.is_open())
                        {
                            getline(eFile, energia);
                            eFile.close();
                            energy.insert({sub, stol(energia)});
                        }
                    }
                    else
                    {
                        for (const auto &entry2 : filesystem::directory_iterator(pathing))
                        {
                            dir = entry2.path().filename();
                            pathing = entry2.path();

                            if (dir.find(cadena) != string::npos)
                            {
                                ruta = pathing + Fname;
                                eFile.open(ruta);
                                if (eFile.is_open())
                                {
                                    getline(eFile, file);
                                    file = file.substr(0, file.find('-'));
                                    eFile.close();
                                }

                                if (file == name)
                                {
                                    ruta = pathing + Fenergy_uj;
                                    eFile.open(ruta);
                                    if (eFile.is_open())
                                    {
                                        getline(eFile, energia);
                                        eFile.close();
                                        energy.insert({sub, stol(energia)});
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return energy;
}

// --- Funciones auxiliares ---
void *execProgram(void *param)
{
    flag = false;
    system((char *)param);
    flag = true;
    return NULL;
}

struct timespec addTimeSpec(struct timespec src1, struct timespec src2)
{
    struct timespec dst;
    dst.tv_sec = src1.tv_sec + src2.tv_sec;
    dst.tv_nsec = src1.tv_nsec + src2.tv_nsec;
    if (dst.tv_nsec >= 1000000000)
    {
        dst.tv_sec += 1;
        dst.tv_nsec -= 1000000000;
    }
    return dst;
}

struct timespec ms2timespec(int src)
{
    struct timespec dst;
    dst.tv_sec = src / 1000;
    dst.tv_nsec = (src % 1000) * 1000000;
    return dst;
}

int timespec2ms(struct timespec src)
{
    return src.tv_sec * 1000 + src.tv_nsec / 1000000;
}

// --- MAIN ---
int main(int argc, char *argv[])
{
    map<string, string> mp = readConfig(argv[1]);
    string param;
    for (int i = 2; i < argc; i++)
        param += string(argv[i]) + " ";
    param = param.substr(0, param.size() - 1);

    // Parámetros
    int sampleT = stoi(mp["SAMPLE_TIME"]);
    int postExecT = stoi(mp["POST_EXEC_TIME"]);
    float minEner = stof(mp["MIN_POWER"]) * 1e-6; // J
    string outputFile = mp["OUTPUT_FILE"];
    string subdomain = mp["SUBDOMAIN"];
    string uso = mp["USO"]; // CPU, GPU, CPU,GPU

    // Preparar tiempos
    struct timespec tdormido = ms2timespec(sampleT);
    struct timespec tactual, tsiguiente, tlimite, tinicio;

    // Variables para CPU
    map<string, long> Eanterior_cpu, Eactual_cpu;
    map<string, float> EnerAcum_cpu;
    map<string, int> repeticiones_cpu;

    // Variables para GPU
    vector<nvmlDevice_t> devices;
    vector<string> gpuNames;
    vector<int> gpuIds;
    map<string, long> Eanterior_gpu;
    map<string, float> EnerAcum_gpu;
    map<string, int> repeticiones_gpu;

    bool monitor_cpu = (uso.find("CPU") != string::npos);
    bool monitor_gpu = (uso.find("GPU") != string::npos);

    // Init NVML solo si monitorizamos GPU
    if (monitor_gpu)
    {
        nvmlInit();
        gpuIds = getGpuIds(mp["GPU"]);
        char nameBuffer[NVML_DEVICE_NAME_BUFFER_SIZE];

        for (int id : gpuIds)
        {
            nvmlDevice_t device;
            if (nvmlDeviceGetHandleByIndex(id, &device) != NVML_SUCCESS)
            {
                cerr << "Error al obtener GPU con ID: " << id << endl;
                continue;
            }
            string name = (nvmlDeviceGetName(device, nameBuffer, NVML_DEVICE_NAME_BUFFER_SIZE) == NVML_SUCCESS) ? string(nameBuffer) : "GPU_" + to_string(id);
            devices.push_back(device);
            gpuNames.push_back(name);
        }
    }

    // Inicializamos energía previa
    if (monitor_cpu)
    {
        Eanterior_cpu = getCpuEnergy_uj(subdomain);
        for (auto &e : Eanterior_cpu)
        {
            EnerAcum_cpu[e.first] = 0.0f;
            repeticiones_cpu[e.first] = 1;
        }
    }
    if (monitor_gpu)
    {
        for (size_t i = 0; i < devices.size(); i++)
        {
            auto energia = getGpuEnergy_uj(devices[i], gpuIds[i], gpuNames[i]);
            for (const auto &e : energia)
            {
                Eanterior_gpu[e.first] = e.second;
                EnerAcum_gpu[e.first] = 0.0f;
                repeticiones_gpu[e.first] = 1;
            }
        }
    }

    // Abrir fichero salida
    ofstream cFile(outputFile, ios::out | ios::trunc);
    cFile << "SUBDOMAIN=" << subdomain << endl;
    cFile << "USO=" << uso << endl;
    cFile << "timestamp(ms);device;id;energy(J);power(W);" << endl;

    clock_gettime(CLOCK_MONOTONIC, &tactual);
    tinicio = tactual;
    tlimite = addTimeSpec(tactual, ms2timespec(postExecT));
    tsiguiente = addTimeSpec(tactual, tdormido);

    pthread_t hilo;
    pthread_create(&hilo, NULL, execProgram, (void *)param.c_str());

    bool noIF = true;
    int i = 0;

    while (!flag || (tlimite.tv_sec > tsiguiente.tv_sec) || ((tlimite.tv_sec == tsiguiente.tv_sec) && (tlimite.tv_nsec >= tsiguiente.tv_nsec)))
    {
        if (noIF)
        {
            clock_gettime(CLOCK_MONOTONIC, &tactual);
            tlimite = addTimeSpec(tactual, ms2timespec(postExecT));
            if (flag)
            {
                noIF = false;
                double postT = (timespec2ms(tactual) - timespec2ms(tinicio)) / 1000.0;
                cout << "POST_EXEC;" << postT << endl;
                cFile << "POST_EXEC;" << postT << endl;
            }
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsiguiente, NULL);
        tsiguiente = addTimeSpec(tsiguiente, tdormido);

        // --- CPU sampling ---
        if (monitor_cpu)
        {
            Eactual_cpu = getCpuEnergy_uj(subdomain);
            for (const auto &e : Eactual_cpu)
            {
                string subdom = e.first;
                long current = e.second;
                long prev = Eanterior_cpu[subdom];
                if (current == prev)
                {
                    repeticiones_cpu[subdom]++;
                    continue;
                }
                float energia_j = (current - prev) * 1e-6f;
                float tiempo_real_seg = (sampleT * repeticiones_cpu[subdom]) / 1000.0f;

                if (energia_j > minEner)
                {
                    float potencia = energia_j / tiempo_real_seg;
                    cFile << i * sampleT << ";CPU;" << subdom << ";" << energia_j << ";" << potencia << ";" << endl;
                    cout << i * sampleT << ";CPU;" << subdom << ";" << energia_j << ";" << potencia << ";" << endl;
                }

                Eanterior_cpu[subdom] = current;
                EnerAcum_cpu[subdom] += energia_j;
                repeticiones_cpu[subdom] = 1;
            }
        }

        // --- GPU sampling ---
        if (monitor_gpu)
        {
            for (size_t j = 0; j < devices.size(); ++j)
            {
                auto energia = getGpuEnergy_uj(devices[j], gpuIds[j], gpuNames[j]);
                for (const auto &e : energia)
                {
                    string subdom = e.first;
                    long current = e.second;
                    long prev = Eanterior_gpu[subdom];
                    if (current == prev)
                    {
                        repeticiones_gpu[subdom]++;
                        continue;
                    }
                    float energia_j = (current - prev) * 1e-6f;
                    float tiempo_real_seg = (sampleT * repeticiones_gpu[subdom]) / 1000.0f;

                    if (energia_j > minEner)
                    {
                        // key = "name;id"
                        size_t pos = subdom.find_last_of(';');
                        string name = subdom.substr(0, pos);
                        string id = subdom.substr(pos + 1);

                        float potencia = energia_j / tiempo_real_seg;
                        cFile << i * sampleT << ";" << name << ";" << id << ";" << energia_j << ";" << potencia << ";" << endl;
                        cout << i * sampleT << ";" << name << ";" << id << ";" << energia_j << ";" << potencia << ";" << endl;
                    }

                    Eanterior_gpu[subdom] = current;
                    EnerAcum_gpu[subdom] += energia_j;
                    repeticiones_gpu[subdom] = 1;
                }
            }
        }

        i++;
    }

    if (monitor_gpu)
        nvmlShutdown();

    cFile.close();
    cout << "FIN" << endl;

    return 0;
}
