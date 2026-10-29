import rclpy
from rclpy.node import Node
import psycopg2

from std_msgs.msg import Bool

#credit https://www.geeksforgeeks.org/python/execute-postgresql-stored-procedure-and-function-in-python/

class QuerySubscriber(Node):

    def __init__(self):
        super().__init__('Query_subscriber')
        self.subscription = self.create_subscription(
            Bool,
            'internet_connection',
            self.listener_callback,
            10)
        self.subscription 

        if msg.data == true 
            query_database()
            

    def query_database(): 
        connector = None
        try: 
            connection_string = "host='host_name' dbname='database_name'\
                        user='user_name' password='your_password'" #place holder data for now
            
            connector = psycopg2.connect(connection_string)

            engine = connector.cursor()

            self.get_logger().info("Database connected.. calling stored procedure..")
            #engine.callproc('get_book_sales', [sale_amount, ]) place holder for calling the pgsql sproc

            self.get_logger().info("Success!")

        except (Exception, psycopg2.DatabaseError) as error: 
            self.get_logger().info("Cant connect to the DB!", error)
        
        finally: 

            if connector
                engine.close()
                connector.close()
                self.get_logger().info("DB connection closed :D")


            


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()