import asyncio

async def fetch_data(name, delay):
    print(f"Start {name}...")
    await asyncio.sleep(delay)   # non-blocking wait
    print(f"Done {name}!")
    return {name: delay}

async def main():
    # Schedule two tasks at once
    task1 = asyncio.create_task(fetch_data("A", 2))
    task2 = asyncio.create_task(fetch_data("B", 4))

    # Await both
    results = await asyncio.gather(task1, task2)
    print("Results:", results)

asyncio.run(main())
