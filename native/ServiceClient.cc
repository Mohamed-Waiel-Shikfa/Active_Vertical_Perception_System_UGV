#include <iostream>
#include <stdexcept>
#include <google/protobuf/text_format.h>
#include <gz/msgs/Factory.hh>
#include <gz/transport/Node.hh>
#include <QJsonDocument>
#include <QJsonObject>

std::string Request(gz::transport::Node &node,const std::string &service,
                    const std::string &requestType,const std::string &replyType,
                    unsigned timeout,const std::string &text)
{
    auto request = gz::msgs::Factory::New(requestType);
    auto reply = gz::msgs::Factory::New(replyType);
    if (!request || !reply || !google::protobuf::TextFormat::ParseFromString(text, request.get()))
      throw std::runtime_error("Unknown protobuf type or invalid request");
    std::string data;
    bool result = false;
    if (!node.RequestRaw(service,request->SerializeAsString(),requestType,replyType,timeout,data,result))
      throw std::runtime_error("Native service timed out");
    if (!result || !reply->ParseFromString(data))
      throw std::runtime_error("Native service rejected request or returned invalid data");
    return reply->DebugString();
}

int main(int argc, char **argv)
{
  if(argc==2 && std::string(argv[1])=="--session")
  {
    gz::transport::Node node;
    for(std::string line;std::getline(std::cin,line);)
    {
      QJsonObject response;
      try
      {
        QJsonParseError error;
        const auto document=QJsonDocument::fromJson(QByteArray::fromStdString(line),&error);
        if(error.error!=QJsonParseError::NoError || !document.isObject())
          throw std::runtime_error("Invalid native request JSON");
        const auto q=document.object();
        const auto text=Request(node,q["service"].toString().toStdString(),
            q["request_type"].toString().toStdString(),q["reply_type"].toString().toStdString(),
            q["timeout_ms"].toInt(),q["text"].toString().toStdString());
        response={{"ok",true},{"text",QString::fromStdString(text)}};
      }
      catch(const std::exception &error)
      {response={{"ok",false},{"error",error.what()}};}
      std::cout<<QJsonDocument(response).toJson(QJsonDocument::Compact).toStdString()<<std::endl;
    }
    return 0;
  }
  if(argc!=6)
  {
    std::cerr<<"usage: greenhouse-service SERVICE REQUEST_TYPE REPLY_TYPE TIMEOUT_MS REQUEST_TEXT\n";
    return 2;
  }
  try
  {
    gz::transport::Node node;
    std::cout<<Request(node,argv[1],argv[2],argv[3],std::stoul(argv[4]),argv[5])<<std::flush;
  }
  catch (const std::exception &error)
  {
    std::cerr << argv[1] << ": " << error.what() << '\n';
    return 1;
  }
}
