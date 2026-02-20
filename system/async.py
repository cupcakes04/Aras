import asyncio

async def fetch_data(name, delay):
    print(f"Start {name}...")
    await asyncio.sleep(delay)   # non-blocking wait
    print(f"Done {name}!")
    return {name: delay}

async def main():
    # Schedule two tasks at once
    task1 = asyncio.create_task(fetch_data("A", 4))
    task2 = asyncio.create_task(fetch_data("B", 6))
    task3 = asyncio.create_task(fetch_data("C", 3))

    # Wait for A first 
    result_3 = await task3 
    print("Got C early:", result_3)

    # Await both
    results_1_2 = await asyncio.gather(task1, task2)
    print("Results:", results_1_2)

asyncio.run(main())
