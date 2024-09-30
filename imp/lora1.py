import time
from machine import Pin, SPI
from lora import LoRa

# Initialize LoRa module
lora = LoRa(
    spi_channel=0,
    spi_speed=8000000
    tx_power=20
)

def main():
    print("LoRa Transmitter")

    while True:
        try:
            my_message = "Hello World, this is Electronic Clinic"
            lora.send(my_message)
            print("Sent:", my_message)
            time.sleep(1)  # Delay for 1 second before sending the next message
        except Exception as e:
            print("Error:", e)

if __name__ == "__main__":
    main()
