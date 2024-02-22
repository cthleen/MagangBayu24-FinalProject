#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Int16Subscriber : public rclcpp::Node {
public:
  Int16Subscriber() : Node("int16_subscriber"), moves_(0) {
    subscription_ = this->create_subscription<std_msgs::msg::Int16>(
      "topic1", 10, std::bind(&Int16Subscriber::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Int16>("topic2", 10);
    publisher2_ = this->create_publisher<std_msgs::msg::String>("topic3", 10);

    std::fill(board_, board_ + 9, 0);
    moves_ = 0;
  }

private:
  // Memeriksa apakah ada yang menang atau tidak
  int check_win(){
    int wins[8][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {0, 3, 6}, {1, 4, 7}, {2, 5, 8}, {0, 4, 8}, {2, 4, 6}};
    for(int i = 0; i < 8; ++i){
      if(board_[wins[i][0]] != 0 && board_[wins[i][0]] == board_[wins[i][1]] && board_[wins[i][1]] == board_[wins[i][2]]){
        return board_[wins[i][2]];
      }
    }
    return 0;
  }

  // Algoritma untuk mencari dan menghitung langkah terbaik yang dapat dilakukan oleh computer
  int minimax(int player){
    int winner = check_win();

    if(winner != 0){
      return winner * player;
    }

    int move = -1;
    int score = -2;

    for(int i = 0; i < 9; i++){
      if(board_[i] == 0){
        board_[i] = player;
        int tempscore = -minimax(player*-1);

        if(tempscore > score){
          score = tempscore;
          move = i;
        }

        board_[i] = 0;
      }
    }
    if(move == -1){
      return 0;
    }
    return score;
  }

  // Menggerakan computer berdasarkan langkah terbaik yang mungkin untuk dilakukan
  int computer_move(){
    int move = -1;
    int score = -2;

    for(int i = 0; i < 9; i++){
      if(board_[i] == 0){
        board_[i] = 1;
        int tempscore = -minimax(-1);
        board_[i] = 0;

        if(tempscore > score){
          score = tempscore;
          move = i;
        }
      }
    }
    return move;
  }

  // Menerima pesan yang berisi langkah pemain serta mengecek dan mengirimkan pesan bila pemain menang, kalah atau seri
  void topic_callback(const std_msgs::msg::Int16::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received data: %d", msg->data);
    

    int k;
    while(moves_ < 9){
      int player = msg->data - 1;

      if(board_[player] == 0){
        board_[player] = -1;

        if(check_win() == -1){
          RCLCPP_INFO(this->get_logger(), "WIN");

          std_msgs::msg::String msg;
          msg.data = "WIN";
          publisher2_->publish(msg);
          break;

        }else if(check_win() == 0 && moves_ == 8){
          RCLCPP_INFO(this->get_logger(), "DRAW");

          std_msgs::msg::String msg;
          msg.data = "DRAW";
          publisher2_->publish(msg);  
          break;
        }

        moves_++;

        if(check_win() == 0){
          k = computer_move();
          board_[k] = 1;

          RCLCPP_INFO(this->get_logger(), "Publishing: %d", k+1);

          if(check_win() == 1){
            RCLCPP_INFO(this->get_logger(), "LOSE");

            std_msgs::msg::String msg;
            msg.data = "LOSE";
            
            publisher2_->publish(msg);
          }

          std_msgs::msg::Int16 computer_msg;
          computer_msg.data = k + 1;
          publisher_->publish(computer_msg);

          moves_++;

          break;
        }
      }
    }
  }

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2_;

  int moves_;
  int board_[9];
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Int16Subscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}