/*
 *     Copyright 2023 RISE Research Institutes of Sweden AB, Safety and Transport   waywise@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#ifndef PARAMETERSERVER_H
#define PARAMETERSERVER_H

#include <QObject>
#include <mutex>
#include <unordered_map>
#include <functional>

class ParameterServer : public QObject
{
    Q_OBJECT
public:
    struct IntParameter {
        std::string name{};
        int32_t value{};
    };
    struct FloatParameter {
        std::string name{};
        float value{};
    };
    struct CustomParameter {
        std::string name{};
        std::string value{};
    };
    struct AllParameters {
        std::vector<IntParameter> intParameters{};
        std::vector<FloatParameter> floatParameters{};
        std::vector<CustomParameter> customParameters{};
    };

    static void initialize();
    static ParameterServer* getInstance();
    bool updateIntParameter(std::string parameterName, int parameterValue);
    bool updateFloatParameter(std::string parameterName, float parameterValue);
    virtual void provideIntParameter(std::string parameterName, std::function<void(int)> setClassParameterFunction, std::function<int(void)> getClassParameterFunction);
    virtual void provideFloatParameter(std::string parameterName, std::function<void(float)> setClassParameterFunction, std::function<float(void)> getClassParameterFunction);
    virtual void saveParametersToXmlFile(QString filename);
    AllParameters getAllParameters();

protected:
    ParameterServer();
    virtual ~ParameterServer(){};
    static ParameterServer *mInstancePtr;
    std::mutex mMutex;
    std::unordered_map<std::string, std::pair<std::function<void(int)>, std::function<int(void)>>> mIntParameterToClassMapping;
    std::unordered_map<std::string, std::pair<std::function<void(float)>, std::function<float(void)>>> mFloatParameterToClassMapping;
};

#endif // PARAMETERSERVER_H
